package frc.robot.subsystems;
// NOTE: Changes to motors, CAN IDs, or aiming logic must be reflected in Theseus/README.md (Turret Aiming section).

//as you may be able to tell, this file was the one that the most time was spent on

//import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.McpJoystick;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ScoringMode;
import static frc.robot.Constants.TurretConfigs.*;

import org.littletonrobotics.junction.Logger;

public class Shooting extends SubsystemBase {
    private Pose2d robotPose;
    private Translation2d turretPosition;
    private Translation2d targetPose;
    private Translation3d turretOffset, limelight2TurretOffset;
    private Rotation3d limelightRotation;
    private Transform3d turret2Robot, limelight2Turret, limelight2Robot;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, rotationSetpoint, heightSetpoint, rotationTau, heightTau, desiredRotation, 
    cachedTargetHeight, desiredHeight, rotationsPerDegree, desiredHeightAngle, turretOffsetX, turretOffsetZ, limelightOffsetX;
    public static double cachedRotationPos;
    private double cachedHeightPos;
    private double cachedFlywheelVel;
    private double cachedThroughborePos;
    private int limelightAcceptedTagID;
    // Per-cycle cached values to avoid redundant computation
    private Translation2d cachedTarget;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, flywheelFolMotor, indexerLMotor, indexerRMotor, feedMotor, vibratorMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final Drivetrain drivetrain;
    private final DoubleArrayPublisher turretTargetPub;
    public final SysIdRoutine flysisid;
    //private final PIDController stinkyPIDcontrollerthatmayormaynotwork;
    private McpJoystick mcpJoystick;
    private boolean turretLocked = false;
    private boolean wasDisabled = true;
    private boolean autoAimEnabled = false;
    private boolean headingHoldMode = false;
    private double heldFieldAngle = 0;
    private boolean lastDashboardAim = false;
    private boolean isShooting = false;
    private double rememberedHeight = 0;
    private boolean mcpShooting = false;
    private boolean mcpAutoAimWasPressed = false;
    private boolean mcpHeadingHoldWasPressed = false;
    private boolean limelightAiming = false;
    private boolean dontchangeheight = false;
    private ScoringMode scoringMode = null;
    private int manualHoldCycles = 0;
    private int manualHeightHoldCycles = 0;
    private static final double MANUAL_STEP_SLOW = 0.1; // fine rotation step for short presses
    private static final double MANUAL_STEP_FAST = 0.5; // fast rotation step after holding ~1s
    private static final double HEIGHT_STEP_SLOW = 0.04;  // fine height step (5:1 ratio needs bigger steps)
    private static final double HEIGHT_STEP_FAST = 0.16;  // fast height step after holding ~1s
    private static final int MANUAL_RAMP_CYCLES = 50;     // cycles before ramping up (~1s at 50Hz)
    private static final double MANUAL_MAX_LEAD = 0.02;   // max rotation setpoint lead over actual position
    private static final double HEIGHT_MAX_LEAD = 0.1;    // max height setpoint lead over actual position
    private static final double STATIONARY_SPEED_THRESHOLD = 0.15; // m/s — below this the robot is "stopped"

    /** @return true if the robot is essentially stationary (translation speed below threshold) */
    private boolean isRobotStationary() {
        var speeds = drivetrain.getChassisSpeeds();
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return speed < STATIONARY_SPEED_THRESHOLD;
    }

    /** Compute the height the motor should actually target right now.
     *  Raises to remembered height when shooting OR stationary; drops to 0 while moving. */
    private double effectiveHeight() {
        return (isShooting || isRobotStationary()) ? rememberedHeight : HEIGHT_REVERSE_LIMIT;
    }

    /** Subsystem for the turret */
    public Shooting(Drivetrain drivetrain) {
        this.drivetrain = drivetrain; // initalize drivetrain
        // Initalize talons
        rotationMotor = new TalonFX(RobotConstants.kTurretRotationID);
        heightMotor =   new TalonFX(RobotConstants.kTurretHeightID);
        flywheelMotor = new TalonFX(RobotConstants.kTurretFlywheelID);
        flywheelFolMotor = new TalonFX(RobotConstants.kTurretFlywheelFolID);
        indexerLMotor =  new TalonFX(RobotConstants.kTurretIndexerID);
        indexerRMotor =  new TalonFX(RobotConstants.kTurretRIndexerID);
        feedMotor =  new TalonFX(RobotConstants.kTurretFeedID);
        vibratorMotor = new TalonFX(RobotConstants.kIntakeVibratorID);
        // Turret motionmagic, a motion profiling that allows for movement from one position to another to be a lot more smooth and seamless
        rotationoutput = new MotionMagicVoltage(0);
        heightoutput =   new MotionMagicVoltage(0);
        targetAngle = 0; //target turret angle
        
        //Set up motors
        rotationMotor.getConfigurator().apply(rotationConfigs);
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply
        indexerLMotor.getConfigurator().apply(indexerConfigs);
        indexerRMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned)); //this does nothing as these commands must be run periodically or alongside the commands that move their master motor otherwise they will do nothing when the master is moving
        flywheelFolMotor.getConfigurator().apply(flyConfigs);
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        robotPose = new Pose2d();
        turretPosition = robotPose.getTranslation();
        
        // Constants
        kmaxVelocity = 2;
        heightTau = 1.0;
        rotationTau = .15;
        desiredRotation = 0;
        rotationsPerDegree = 1.45 / (26 - 67);
        turretOffsetZ = .402;
        turretOffsetX = -.105;
        limelightOffsetX = .2;

        //Set offsets for the limelight
        LimelightHelpers.setFiducial3DOffset("limelight-stinky", 0, -.15, -.58); //so the limelight knows where it is initially on the robot

        // Operator Stuff
        turretTargetPub = NetworkTableInstance.getDefault()
            .getTable("Pose").getDoubleArrayTopic("turretTarget").publish(); 
        SmartDashboard.putData("Zero Turret", new InstantCommand(this::zeroTurret).ignoringDisable(true));
        SmartDashboard.putBoolean("Velocity Compensation", false);
        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled);
        SmartDashboard.putNumber("Shoot Speed", 1.0);

        // Currently usused sysid rougine for the flywheels
        //This moves the motor which is called in the function as a specific voltage and (I could be wrong) uses the measured resistances to generate the feedback and feedforward values with internal math that allow the motors to move the smoothest and most accurate
        flysisid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                state -> SignalLogger.writeString("SysIdFlywheels_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> {flywheelMotor.setVoltage(volts.in(Volts)); flywheelFolMotor.setVoltage(volts.in(Volts));},
                null,
                this
            )
        );
    }

    //Function that runs peridocaly while the Subsystem is active
    @Override
    public void periodic() {
        robotPose = drivetrain.getState().Pose; //updates a variable in this file to equal a nonstatic variable in another file, could have been done more efficiently most likely but this was easiest
        turretPosition = robotPose.transformBy(
            new Transform2d(new Translation2d(RobotConstants.kTurretXOffsetMeters, 0), new Rotation2d()))
            .getTranslation();
        // Only react to the dashboard toggle when the operator actually
        // clicked it (value differs from what we last wrote).
        boolean dashboardAim = SmartDashboard.getBoolean("Auto Aim", autoAimEnabled); //checks if the button has been clicked on Smartdashboard/Elastic
        if (dashboardAim != lastDashboardAim) {
            autoAimEnabled = dashboardAim;
        }

        // Cache CAN reads once per cycle
        cachedRotationPos = rotationMotor.getPosition().getValueAsDouble(); //stores the positions of the motors
        cachedHeightPos = heightMotor.getPosition().getValueAsDouble();
        cachedFlywheelVel = flywheelMotor.getVelocity().getValueAsDouble();

        if (limelightAiming != true) { //this was for if we got localization working and never got used
            LimelightHelpers.setPipelineIndex("limelight-stinky", 1); //changes the settings of the camera, for localization I wanted the limelight to record fromt the apriltag while for limelight autoaim I wanted it to record from half a meter behidn the apriltag
            targetAim();
        }
        if (limelightAiming == true) { //could have been an else statement
            LimelightHelpers.setPipelineIndex("limelight-stinky", 0);
            limelightAim();
        }
        // Check MCP simulated joystick for height/rotation/shoot commands, the mcpJoystick is part of why manual aiming never got better as the head technician did not understand it
        if (mcpJoystick != null && mcpJoystick.isActive() && DriverStation.isEnabled()) {
            int mcpPov = mcpJoystick.getPOV();
            if (mcpPov == 0) manualHeight(1);        // D-pad up
            else if (mcpPov == 180) manualHeight(-1); // D-pad down
            else if (mcpPov == 270) manualRotate(-1); // D-pad left
            else if (mcpPov == 90) manualRotate(1);   // D-pad right

            // Button 1 (A) toggles auto-aim on rising edge AKA when pressed not when released
            boolean mcpAPressed = mcpJoystick.getButton(1);
            if (mcpAPressed && !mcpAutoAimWasPressed) {
                if (autoAimEnabled) { autoAimEnabled = false; } else { enableAutoAim(); }
            }
            mcpAutoAimWasPressed = mcpAPressed;

            // Button 2 (B) toggles heading-hold on rising edge
            boolean mcpBPressed = mcpJoystick.getButton(2);
            if (mcpBPressed && !mcpHeadingHoldWasPressed) {
                toggleHeadingHold();
            }
            mcpHeadingHoldWasPressed = mcpBPressed;

            // Right trigger (axis 3) > 0.5 = shoot
            if (mcpJoystick.getAxis(3) > 0.5) {
                if (!mcpShooting) { shoot(); mcpShooting = true; }
            } else if (mcpShooting) {
                unshoot(); mcpShooting = false;
            }
        } else if (mcpShooting) {
            unshoot(); mcpShooting = false;
        }

        //its commented out for a reason, basically tried to have a dynamic limelight position for localization, did not work
       /*  try {
        turretOffset = new Translation3d(turretOffsetX, 0, turretOffsetZ);
        limelight2TurretOffset = new Translation3d(limelightOffsetX, 0, 0);
        limelightRotation = new Rotation3d(0, Math.toRadians(18), Math.toRadians(getCurRotation()));
        turret2Robot = new Transform3d(turretOffset, new Rotation3d());
        limelight2Turret = new Transform3d(limelight2TurretOffset, limelightRotation);
        if (!Double.isNaN(getCurRotation())) {
            limelight2Robot = turret2Robot.plus(limelight2Robot);
        }
    } catch (Exception e) {
    }
        if (limelight2Robot != null) {
            LimelightHelpers.setCameraPose_RobotSpace(
                "limelight-stinky",
                limelight2Robot.getX(),
                limelight2Robot.getY(),
                limelight2Robot.getZ(),
                Math.toDegrees(limelight2Robot.getRotation().getX()),
                Math.toDegrees(limelight2Robot.getRotation().getY()),
                Math.toDegrees(limelight2Robot.getRotation().getZ())
            );
        }
        System.out.println(LimelightHelpers.getCameraPose3d_RobotSpace("limelight-stinky")); */

        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled); //logging
        lastDashboardAim = autoAimEnabled;
        SmartDashboard.putBoolean("Heading Hold", headingHoldMode);
        SmartDashboard.putBoolean("Turret Manual", !autoAimEnabled && !headingHoldMode);
        SmartDashboard.putString("Scoring Mode", scoringMode != null ? scoringMode.name() : "NONE");
        SmartDashboard.putNumber("Turret Angle", cachedRotationPos * 360.0);
        //if (Math.abs(cachedRotationPos * 360.0) < rotationSetpoint - .1) {feedMotor.stopMotor(); flywheelMotor.stopMotor(); indexerLMotor.stopMotor(); indexerRMotor.stopMotor();}
        if (turretPosition != null) {
            turretTargetPub.set(new double[] {
                targetPose != null ? targetPose.getX() : 0,
                targetPose != null ? targetPose.getY() : 0, 0 });
        }

        Logger.recordOutput("Turret/RotationPosition", cachedRotationPos);
        Logger.recordOutput("Turret/HeightPosition", cachedHeightPos);
        Logger.recordOutput("Turret/HeightSetpoint", heightSetpoint);
        SmartDashboard.putNumber("heightstepoint", heightSetpoint);
        Logger.recordOutput("Turret/RememberedHeight", rememberedHeight);
        Logger.recordOutput("Turret/IsShooting", isShooting);
        Logger.recordOutput("Turret/RotationSetpoint", rotationSetpoint);
        Logger.recordOutput("Turret/FlywheelVelocity", cachedFlywheelVel);
        Logger.recordOutput("Turret/ThroughborePosition", cachedThroughborePos);
        Logger.recordOutput("Turret/TargetAngle", Math.toDegrees(targetAngle));
        Logger.recordOutput("Turret/DesiredMotorRotation", desiredRotation);
        Logger.recordOutput("Turret/HeadingHoldMode", headingHoldMode);
        Logger.recordOutput("Turret/ScoringMode", scoringMode != null ? scoringMode.name() : "NONE");
        SmartDashboard.putNumber("desiredRotation", desiredRotation);
        
        if (turretPosition != null) {
            Logger.recordOutput("Turret/PositionX", turretPosition.getX());
            Logger.recordOutput("Turret/PositionY", turretPosition.getY());
        }

        if (robotPose != null && turretPosition != null) {
            double robotHeading = robotPose.getRotation().getRadians();

            double actualFieldAngle = robotHeading - cachedRotationPos * 2 * Math.PI;
            Logger.recordOutput("Turret/ActualPose", new Pose2d(turretPosition, new Rotation2d(actualFieldAngle)));

            double desiredFieldAngle = robotHeading - targetAngle;
            Logger.recordOutput("Turret/DesiredPose", new Pose2d(turretPosition, new Rotation2d(desiredFieldAngle)));
        }
    }

    /** Set the MCP joystick reference for simulated input. */
    public void setMcpJoystick(McpJoystick mcp) {
        this.mcpJoystick = mcp;
    }

    /** 
     * Gets the target field position based on the alliance and current position
     * @return The Translation2d of where it should be
    */
    public Translation2d targetpose() { //this entire function was not used but uses field coordinates pulled from pathplanner as to where the fuel should land based on where the robot is on the field
        targetPose = new Translation2d(0, 0);
        //if on red alliance
        if (Drivetrain.getAlliance()) {
            limelightAcceptedTagID = 10;
            //red alliance hub area
            if (robotPose.getX() > 12.5) {
                return targetPose = new Translation2d(11.914, 4.035);
            }
            //not red alliance hub area
            else if (robotPose.getX() < 12.5) {
                //upper portion of field
                if (robotPose.getY() > 4) {
                    return targetPose = new Translation2d(15, 7);
                }
                //lower portion of field
                else if (robotPose.getY() < 4.425) {
                    return targetPose = new Translation2d(15, 1);
                }
            }
        }
        //if on blue alliance
        else if (!Drivetrain.getAlliance()) {
            limelightAcceptedTagID = 26;
            //blue alliance hub area
            if (robotPose.getX() < 4.425) {
                return targetPose = new Translation2d(4.621, 4.035);
            } 
            //not blue alliance hub area
            else if (robotPose.getX() > 4) {
                //upper portion of field
                if (robotPose.getY() > 4) {
                    return targetPose = new Translation2d(2, 7);
                }
                //lower portion of field
                else if (robotPose.getY() < 4) {
                    return targetPose = new Translation2d(2, 1);
                }
            }
        }
        return targetPose;
    }

    /**Gets the height of the target based on which target it is aiming at, and subtracts the robot height to get the actual height */
    public double getTargetHeight(){
        if (robotPose.getX() < 4 || robotPose.getX() > 12.5) {
            return 1.8288 - turretHeight;
        }
        else {
            return turretHeight;
        }
    }

    /**Calculating the actual angle while the robot is moving*/ //This always ended up adjusting way more than it needed to and was never tuned or fixed so the math is likely wrong
    public double vectorCalculations() {
        targetx = (cachedTarget.getX() - turretPosition.getX()); 
        targety = (cachedTarget.getY() - turretPosition.getY());
        targetAngle = (Math.atan2(targety, targetx));
        boolean velocityCompensation = SmartDashboard.getBoolean("Velocity Compensation", false);
        if (velocityCompensation) {
            double shootingXSpeed = kmaxVelocity*Math.cos(Math.toRadians(65)); //basic vectors and trigonometry
            // Add robot velocity so the projectile leads the target
            double actualX = (shootingXSpeed * Math.cos(targetAngle));
            double actualY = (shootingXSpeed * Math.sin(targetAngle)) + (drivetrain.getChassisSpeeds().vyMetersPerSecond);
            targetAngle = Math.atan2(actualY, actualX);
        }
        // Convert from field-relative to robot-relative
        targetAngle -= drivetrain.getState().Pose.getRotation().getRadians(); //adjust the angle that is needed to be robot relative
        // Normalize to [-π, π) so 0 = front of robot
        targetAngle = -Math.IEEEremainder(targetAngle, 2 * Math.PI);
        double smartdashboardangle = Math.toDegrees(targetAngle); //this is just because very few people can read radians as a decimal and know what angle that is
        SmartDashboard.putNumber("turret expected angle", smartdashboardangle);
        return (targetAngle) / (2 * Math.PI);
    }
    /**
     * Kinematics used to figure out the angle
     * Max definetly wrote it trust
    */
    public double maxFormula(){ //Doug's AI made everything into shortened variables, the numbers is just the power to which that variable is
        targetx = (cachedTarget.getX() - turretPosition.getX()); //error between current and wanted x position
        targety = (cachedTarget.getY() - turretPosition.getY()); // error between current and wanted y position
        targetDistance = Math.hypot(targetx, targety); //find the error between current and wanted 2D coordinate
        double v2 = kmaxVelocity * kmaxVelocity; //velocity squared as the localization was supposed to only adjust turret angle and keep speed the same
        double v4 = v2 * v2; //velocity to the 4th
        double d2 = targetDistance * targetDistance; //distance squared
        double g = 9.80665; //the acceleration due to gravity, should probably be negative I don't know
        double h = cachedTargetHeight; //height
        double discriminant = d2 - (2 * g * targetDistance) * ((h / v2) + ((g * d2) / (2 * v4))); //for those who don't know a discriminant is just whatever goes inside the square root, if this is negative then the square root will return with an error as we have no use for imaginary numbers
        if (discriminant >= 0) {
            return Math.atan2((targetDistance + Math.sqrt(discriminant)), (g * d2 / v2)) / (2 * Math.PI); //its just some advanced kinematics which substitutes time and finds an angle for a specific spot on a 2D plane
        }
        else {
            return 67; //this is not a joke, this is the initial angle that the turret rests at measured with the measure app on my Iphone
        }
    }
    /**Moves the turret to the wanted spots*/
    public void targetAim(){
        if (robotPose == null || turretPosition == null) return;

        // Don't send motor commands while disabled — it causes the turret to
        // wind up and slam on first enable.
        if (!DriverStation.isEnabled()) {
            wasDisabled = true;
            return;
        }

        // On the first enabled cycle, sync the smoothing state to the
        // actual motor positions so there's no accumulated jump.
        if (wasDisabled) {
            rotationSetpoint = cachedRotationPos;
            heightSetpoint = cachedHeightPos;
            rememberedHeight = cachedHeightPos;
            wasDisabled = false;
        } 

        // --- Heading-hold mode: maintain the captured field angle using only gyro ---
        if (headingHoldMode) {
            double heading = robotPose.getRotation().getRadians();
            double robotRelative = Math.IEEEremainder(heldFieldAngle - heading, 2 * Math.PI); //adjusts the angle so that 0 is always the front of the robot and not the front of the field
            desiredRotation = -robotRelative / (2 * Math.PI); //rotation in amount of rotations
            desiredRotation = MathUtil.clamp(desiredRotation, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT); //stops the value from exceeding thresholds that would snap either a wire or a 3D printed part

            double rotError = desiredRotation - rotationSetpoint;
            rotationSetpoint += rotationTau * rotError; //rotationTau is just an arbitrary small number
            rotationSetpoint = MathUtil.clamp(rotationSetpoint, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
            //if (Math.abs(hErr) > 0.01) heightSetpoint += heightTau * hErr; I think this line made a lot of errors
            heightSetpoint = MathUtil.clamp(heightSetpoint, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
            if (scoringMode == ScoringMode.PASSING) {heightMotor.setControl(heightoutput.withPosition(1.45));} //since this stuff didn't actually move the hood, this sets the hood all the way up if we are passing
            else {heightMotor.setControl(heightoutput.withPosition(0));} //hood all the way down if shooting
            if (dontchangeheight == true) {heightMotor.stopMotor();} //this stops the hood from moving if we want it to hold
            rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
            return; //don't do any auto aiming
        }

        // --- Normal auto-aim path ---
        // Compute target once per cycle — used by vectorCalculations() and maxFormula(), this never worked
        cachedTarget = targetpose();
        cachedTargetHeight = getTargetHeight();
        desiredHeightAngle = 90 - maxFormula();
        desiredHeight = (desiredHeightAngle - 23) * rotationsPerDegree; // 23 is empirically determined "zero" height angle
        //System.out.println(desiredHeight);
        desiredRotation  = vectorCalculations();
        // Clamp to soft limits (±135° from front-of-robot zero)
        desiredRotation = MathUtil.clamp(desiredRotation, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);

        if (turretLocked || !autoAimEnabled) {
            //if (Math.abs(hErr) > 0.01) heightSetpoint += heightTau * hErr;
            heightSetpoint = MathUtil.clamp(heightSetpoint, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
            rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
            return;
        }

        double rotError = desiredRotation - rotationSetpoint;
        rotationSetpoint += rotationTau * rotError;
        rotationSetpoint = MathUtil.clamp(rotationSetpoint, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        
        // Track the auto-aim height so it's ready when shooting starts
        desiredHeight = MathUtil.clamp(desiredHeight, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        double remErr = desiredHeight - rememberedHeight;
        if (Math.abs(remErr) > 0.01) rememberedHeight += heightTau * remErr;
        rememberedHeight = MathUtil.clamp(rememberedHeight, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);

        double heightError = effectiveHeight() - heightSetpoint;
        if (Math.abs(heightError) > .01) {
            heightSetpoint = heightError;
        }
        heightSetpoint = MathUtil.clamp(heightSetpoint, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
    }

    /** Makes use of the LimeLight 4 mounted to the turret to aim at the correct apriltag */
    public void limelightAim() { //this worked so well it could knock your socks off
        //isLimelightAiming = !isLimelightAiming;
        //if (isLimelightAiming == false) return;
        LimelightHelpers.setPipelineIndex("limelight-stinky", 1);
        if (robotPose == null || turretPosition == null) return; //ignore if the position of the turret was unknown or if the robot has no location
        if (!LimelightHelpers.getTV("limelight-stinky")) {return;} //ignore if there is no valid targets to aim at
        if (wasDisabled) {
            rotationSetpoint = cachedRotationPos;
            heightSetpoint = cachedHeightPos;
            rememberedHeight = cachedHeightPos;
            wasDisabled = false;
        }
        limelightAiming = true;
        double TX = 0;
        double TY = 0;
        LimelightHelpers.setPriorityTagID("limelight-stinky", limelightAcceptedTagID); //basically if this tag is in frame then aim for it and ignore any other apriltag in frame
        TX = LimelightHelpers.getTX("limelight-stinky"); //the distance between the crosshare and the apriltag in degrees
        TY = Math.IEEEremainder(TX, 360); //no full revolutions
        TY = TX / 360.0; //put it in terms of turret rotations
        TY = MathUtil.clamp(TY, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT); //don't break stuff by exceeding these thresholds
        if (Math.abs(TY) > .0){
            rotationSetpoint += (.1 * TY); //step up towards the target, basically a manual P in PID
        }

        if (rotationSetpoint > ROTATION_FORWARD_LIMIT) {
            rotationSetpoint = ROTATION_REVERSE_LIMIT;
        } //loop around so that it is still continuous with its tracking
        if (rotationSetpoint < ROTATION_REVERSE_LIMIT) {
            rotationSetpoint = ROTATION_FORWARD_LIMIT;
        }
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint)); //move the motor to the target
        SmartDashboard.putNumber("desiredrotation", TY); //debugging
        double ranging = Math.hypot(LimelightHelpers.getBotPose3d_TargetSpace("limelight-stinky").getX(), 
        LimelightHelpers.getBotPose3d_TargetSpace("limelight-stinky").getZ() - .5); //gets the distance from the robot to half a meter behind the apriltag using coordinates where the apriltag is (0, 0)
        double g = 9.80665; //gravity constant
        System.out.println(LimelightHelpers.getCameraPose3d_TargetSpace("limelight-stinky").getZ()); //debugging
        //double speed = ranging / Math.cos(Math.toRadians(70)) * Math.sqrt(g / (2 * (Math.abs((ranging * Math.tan(Math.toRadians(70))) - (1.8288 - turretHeight)))));
        double speed = Math.sqrt((ranging * g) / (Math.sin(Math.toRadians(66) * 2))); //kinematics substituting for time
        //System.out.println(ranging);
        //System.out.println(speed / (Math.PI * 2.8));
        flywheelMotor.set(speed / (Math.PI * 2.4)); //supposed to divide by the circumferance of the wheel but this value was a lot more accurate even though the diameter of the wheel is 4 inches
        flywheelFolMotor.set(speed / (Math.PI * 2.4));
        if (speed > .1) { //shoot if it is actually going to be able to spit out a fuel
            indexerLMotor.setVoltage(-12);
            feedMotor.setVoltage(-12);
            indexerRMotor.setVoltage(12);
            vibratorMotor.set(.4);
        }
    }

    /** Duplicate of the {@code limelightAim()} function, however altered to be better suited for use in Autonomous mode */ //see above for what it does
    public void autolimelightAim() {
        //isLimelightAiming = !isLimelightAiming;
        //if (isLimelightAiming == false) return;
        LimelightHelpers.setPipelineIndex("limelight-stinky", 1);
        if (robotPose == null || turretPosition == null) return;
        if (!LimelightHelpers.getTV("limelight-stinky")) {return;}
        if (wasDisabled) {
            rotationSetpoint = cachedRotationPos;
            heightSetpoint = cachedHeightPos;
            rememberedHeight = cachedHeightPos;
            wasDisabled = false;
        }
        limelightAiming = true;
        double TX = 0;
        double TY = 0;
        LimelightHelpers.setPriorityTagID("limelight-stinky", limelightAcceptedTagID);
        TX = LimelightHelpers.getTX("limelight-stinky");
        TY = Math.IEEEremainder(TX, 360);
        TY = TX / 360.0;
        TY = MathUtil.clamp(TY, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        if (Math.abs(TY) > .001) {
            rotationSetpoint += (.2 * TY);
        }

        if (rotationSetpoint > ROTATION_FORWARD_LIMIT) {
            rotationSetpoint = ROTATION_REVERSE_LIMIT;
        }
        if (rotationSetpoint < ROTATION_REVERSE_LIMIT) {
            rotationSetpoint = ROTATION_FORWARD_LIMIT;
        }
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
        SmartDashboard.putNumber("desiredrotation", TY);
        double ranging = Math.hypot(LimelightHelpers.getBotPose3d_TargetSpace("limelight-stinky").getX(), 
        LimelightHelpers.getBotPose3d_TargetSpace("limelight-stinky").getZ() - .5);
        double g = 9.80665;
        System.out.println(LimelightHelpers.getCameraPose3d_TargetSpace("limelight-stinky").getZ());
        //double speed = ranging / Math.cos(Math.toRadians(70)) * Math.sqrt(g / (2 * (Math.abs((ranging * Math.tan(Math.toRadians(70))) - (1.8288 - turretHeight)))));
        double speed = Math.sqrt((ranging * g) / (Math.sin(Math.toRadians(66) * 2)));
        //System.out.println(ranging);
        //System.out.println(speed / (Math.PI * 2.8));
        flywheelMotor.set(speed / (Math.PI * 2.4));
        flywheelFolMotor.set(speed / (Math.PI * 2.4));
        if (speed > .1) {
            indexerLMotor.setVoltage(-12);
            feedMotor.setVoltage(-12);
            indexerRMotor.setVoltage(12);
            vibratorMotor.set(.4);
        }
    }

    /**The shoot function makes the robot shoot. */
    public void shoot(){
        flywheelMotor.set(.7);
        flywheelFolMotor.set(.7);
        limelightAiming = false;
        isShooting = true;
    }
    /** Begins to intake Fuel towards the flywheel*/
    public void index() {
        limelightAiming = false;
        isShooting = true;
        if (scoringMode == ScoringMode.PASSING) {flywheelMotor.setVoltage(12);flywheelFolMotor.setVoltage(12); heightMotor.setControl(heightoutput.withPosition(1.45));}
        else {flywheelMotor.set(.6);flywheelFolMotor.set(.6); heightMotor.setControl(heightoutput.withPosition(0));}
        if (autoAimEnabled) {
            double g = 9.80665;
            double speed = Math.sqrt((targetDistance * g) / (Math.sin(Math.toRadians(67) * 2)));
            flywheelMotor.set(speed);
            flywheelFolMotor.set(speed);
        }
        indexerLMotor.setVoltage(-10);
        feedMotor.setVoltage(-10); 
        indexerRMotor.setVoltage(10);
        vibratorMotor.set(.4);
    }
    /** {@code shoot()} but for Autonomous */ //the code crashed if there were not seperate functions for autonomous and teleop. no clue why
    public void autoshoot(){
        isShooting = true;
        flywheelMotor.set(.6);
        flywheelFolMotor.set(.6);
    }
    /** {@code index()} but for Autonomous */
    public void autoindex(){
        isShooting = true;
        flywheelMotor.set(.51);
        flywheelFolMotor.set(.51);
        indexerLMotor.setVoltage(-8);
        feedMotor.setVoltage(-8);
        indexerRMotor.setVoltage(8);
        vibratorMotor.set(.4);
    }
    /** Stops shooting */
    public void unshoot(){
        isShooting = false;
        limelightAiming = false;
        flywheelMotor.stopMotor();
        flywheelFolMotor.stopMotor();
        indexerLMotor.stopMotor();
        feedMotor.stopMotor();
        indexerRMotor.stopMotor();
        vibratorMotor.stopMotor();

    }
     public void lockTurret(){ //this was supposed to turn off autoaim but instead ended up being the function to raise the hood
        //controller.setPID(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);
        //controller2.setPID(RobotConstants.kTurretHeightP, RobotConstants.kTurretHeightI, RobotConstants.kTurretHeightD);
        //rotationMotor.setControl(rotationoutput.withPosition(0));
        heightMotor.setControl(heightoutput.withPosition(1.45));
    }
   
    /** Resets the angle of the turret to 0 */
    public void resetTurret(){
        heightMotor.setControl(heightoutput.withPosition(0));
    }
    /**Reverses the direction of the Indexer */
    public void reverseIndexer() { // this was useful for when fuel was jammed
        indexerLMotor.set(1);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
    }
    /** Roughly gets the curret rotation of the turret 
     * @return The rotation of the motor in degrees
    */
    public Double getCurRotation() { //this was used for the LL4 localization and is no longer used
        return rotationMotor.getPosition().getValueAsDouble() * 360.0;
    }
    /** Nudge the turret manually. Automatically switches to manual mode.
     * Starts slow for precision, ramps up after holding ~1 second.
     * @param direction +1 for right, -1 for left */
    public void manualRotate(double direction) { //I can not explain this because however I thought it worked seemed to be wrong
        autoAimEnabled = false;
        headingHoldMode = false;
        manualHoldCycles++;
        double t = Math.min(5.0, (double) manualHoldCycles / MANUAL_RAMP_CYCLES);
        double step = MANUAL_STEP_SLOW + (MANUAL_STEP_FAST - MANUAL_STEP_SLOW) * t * 10;
        double desired = rotationSetpoint + step * direction;
        desired = MathUtil.clamp(desired, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        // Don't let the setpoint outrun the actual motor position
        double lead = desired - cachedRotationPos;
        if (Math.abs(lead) > MANUAL_MAX_LEAD) {
            desired = cachedRotationPos + Math.copySign(MANUAL_MAX_LEAD, lead);
        }
        rotationSetpoint = desired;
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
    }

    /** Nudge the turret height manually. Automatically switches to manual mode.
     * Starts slow for precision, ramps up after holding ~1 second.
     * @param direction +1 for up, -1 for down */
    public void manualHeight(double direction) {
        autoAimEnabled = false;
        headingHoldMode = false;
        limelightAiming = false;
        manualHeightHoldCycles++;
        double t = Math.min(1.0, (double) manualHeightHoldCycles / MANUAL_RAMP_CYCLES);
        double step = HEIGHT_STEP_SLOW + (HEIGHT_STEP_FAST - HEIGHT_STEP_SLOW) * t;
        // Start from actual position if setpoint is out of sync
        double base = Math.abs(rememberedHeight - cachedHeightPos) > HEIGHT_MAX_LEAD
                    ? cachedHeightPos : rememberedHeight;
        double desired = base + step * direction;
        desired = MathUtil.clamp(desired, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        double lead = desired - cachedHeightPos;
        if (Math.abs(lead) > HEIGHT_MAX_LEAD) {
            desired = cachedHeightPos + Math.copySign(HEIGHT_MAX_LEAD, lead);
        }
        rememberedHeight = desired;
        heightSetpoint = effectiveHeight();
        heightMotor.setControl(heightoutput.withPosition(heightSetpoint));
    }

    /** Resets the manual ramp counters (call when D-pad is released) */
    public void resetManualRamp() {
        manualHoldCycles = 0;
        manualHeightHoldCycles = 0;
    }

    /** Switch back to auto-aim mode */
    public void enableAutoAim() {
        autoAimEnabled = true;
        limelightAiming = true;
        headingHoldMode = false;
    }

    /** Disable all automatic turret modes (auto-aim, heading-hold, scoring preset).
     *  Turret stays at its current position in manual mode. */
    public void disableAllModes() {
        autoAimEnabled = false;
        headingHoldMode = false;
        scoringMode = null;
    }
    /** Toggle heading-hold mode: turret maintains its current field-relative
     *  angle using only the gyro, ignoring AprilTag-corrected field position. */
    public void toggleHeadingHold() { //see targetAim()
        if (headingHoldMode) {
            headingHoldMode = false;
        } else {
            double heading = robotPose != null ? robotPose.getRotation().getRadians() : 0; 
            heldFieldAngle = heading - rotationSetpoint * 2 * Math.PI;
            headingHoldMode = true;
            dontchangeheight = true;
            autoAimEnabled = false;
            turretLocked = false;
            scoringMode = null; // clear scoring/passing preset
        }
    }

    /** Toggle between Scoring and Passing presets. Each preset activates
     *  heading-hold at a fixed field angle and sets the height + flywheel speed. */
    public void toggleScoringMode() {
        if (scoringMode == null || scoringMode == ScoringMode.PASSING) {
            scoringMode = ScoringMode.SCORING;
             dontchangeheight = false;
        } else {
            scoringMode = ScoringMode.PASSING;
            dontchangeheight = false;
        }
        // Activate heading-hold at the mode's field angle
        boolean isRed = Drivetrain.getAlliance();
        heldFieldAngle = isRed ? scoringMode.redFieldAngle : scoringMode.blueFieldAngle;
        headingHoldMode = true;
        autoAimEnabled = false;
        turretLocked = false;
        rememberedHeight = scoringMode.height;
        SmartDashboard.putNumber("Shoot Speed", scoringMode.flywheelSpeed);
    }

    /** Zero the turret rotation and height encoders at current physical position */
    public void zeroTurret() {
        autoAimEnabled = false;
        rotationMotor.setPosition(0);
        rotationSetpoint = 0;
        rotationMotor.setControl(rotationoutput.withPosition(0));
        heightMotor.setPosition(0);
        heightSetpoint = 0;
        rememberedHeight = 0;
        heightMotor.setControl(heightoutput.withPosition(0));
    }

    /** @return true if turret is in manual mode */
    public boolean isManualMode() {
        return !autoAimEnabled;
    }

    /**Stops all turret related motors, the Rotation, Height, and Indexer motors */
    public void stopMotors(){
        limelightAiming = false;
        turretLocked = false;
        rotationMotor.stopMotor();
        heightMotor.stopMotor();
        indexerLMotor.stopMotor();
        indexerRMotor.stopMotor();
        feedMotor.stopMotor();
        vibratorMotor.stopMotor();
    }
}