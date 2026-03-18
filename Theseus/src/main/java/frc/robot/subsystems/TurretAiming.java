package frc.robot.subsystems;
// NOTE: Changes to motors, CAN IDs, or aiming logic must be reflected in Theseus/README.md (Turret Aiming section).

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.McpJoystick;
import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.TurretConfigs.*;

import org.littletonrobotics.junction.Logger;

public class TurretAiming extends SubsystemBase {
    private Pose2d robotPose;
    private Translation2d turretPosition;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, 
    rotationSetpoint, heightSetpoint, rotationTau, heightTau, desiredRotation;
    // Per-cycle cached values to avoid redundant computation
    private Translation2d cachedTarget;
    private double cachedTargetHeight;
    private double cachedRotationPos, cachedHeightPos, cachedFlywheelVel, cachedThroughborePos;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, indexerLMotor, indexerRMotor, feedMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleArrayPublisher turretTargetPub;
    //private final PIDController stinkyPIDcontrollerthatmayormaynotwork;
    private final DutyCycleEncoder throughbore;
    private McpJoystick mcpJoystick;
    private boolean turretLocked = false;
    private boolean wasDisabled = true;
    private boolean autoAimEnabled = false;
    private boolean lastDashboardAim = false;
    private boolean mcpShooting = false;
    private boolean mcpAutoAimWasPressed = false;
    private int manualHoldCycles = 0;
    private int manualHeightHoldCycles = 0;
    private static final double MANUAL_STEP_SLOW = 0.002; // fine rotation step for short presses
    private static final double MANUAL_STEP_FAST = 0.008; // fast rotation step after holding ~1s
    private static final double HEIGHT_STEP_SLOW = 0.01;  // fine height step (5:1 ratio needs bigger steps)
    private static final double HEIGHT_STEP_FAST = 0.04;  // fast height step after holding ~1s
    private static final int MANUAL_RAMP_CYCLES = 50;     // cycles before ramping up (~1s at 50Hz)
    private static final double MANUAL_MAX_LEAD = 0.02;   // max rotation setpoint lead over actual position
    private static final double HEIGHT_MAX_LEAD = 0.1;    // max height setpoint lead over actual position

    /** Subsystem for the turret */
    public TurretAiming(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotationMotor = new TalonFX(RobotConstants.kTurretRotationID);
        heightMotor =   new TalonFX(RobotConstants.kTurretHeightID);
        flywheelMotor = new TalonFX(RobotConstants.kTurretFlywheelID);
        indexerLMotor =  new TalonFX(RobotConstants.kTurretIndexerID);
        indexerRMotor =  new TalonFX(RobotConstants.kTurretRIndexerID);
        feedMotor =  new TalonFX(RobotConstants.kTurretFeedID);
        rotationoutput = new MotionMagicVoltage(0);
        heightoutput =   new MotionMagicVoltage(0);
        throughbore = new DutyCycleEncoder(2, 1, 0.216);
        targetAngle = 0;
        robotPose = new Pose2d();
        turretPosition = robotPose.getTranslation();
        kmaxVelocity = 2;
        heightTau = .15;
        rotationTau = .15;
        desiredRotation = 0;
        turretTargetPub = NetworkTableInstance.getDefault()
            .getTable("Pose").getDoubleArrayTopic("turretTarget").publish();
        //Set up motors
        rotationMotor.getConfigurator().apply(rotationConfigs);
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply
        indexerLMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        //stinkyPIDcontrollerthatmayormaynotwork = new PIDController(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);

        SmartDashboard.putData("Zero Turret", new InstantCommand(this::zeroTurret).ignoringDisable(true));
        SmartDashboard.putBoolean("Velocity Compensation", false);
        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled);
        SmartDashboard.putNumber("Shoot Speed", 1.0);
    }

    /** Set the MCP joystick reference for simulated input. */
    public void setMcpJoystick(McpJoystick mcp) {
        this.mcpJoystick = mcp;
    }

    /** 
     * Gets the target field position based on the alliance and current position
     * @return The Translation2d of where it should be
    */
    public Translation2d targetpose() {
        targetPose = new Translation2d(0, 0);
        //if on red alliance
        if (CommandSwerveDrivetrain.getAlliance()) {
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
        else if (!CommandSwerveDrivetrain.getAlliance()) {
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

    /**Calculating the actual angle while the robot is moving*/
    public double vectorCalculations() {
        targetx = (cachedTarget.getX() - turretPosition.getX()); 
        targety = (cachedTarget.getY() - turretPosition.getY());
        targetAngle = (Math.atan2(targety, targetx));
        boolean velocityCompensation = SmartDashboard.getBoolean("Velocity Compensation", false);
        if (velocityCompensation) {
            double shootingXSpeed = kmaxVelocity*Math.cos(maxFormula());
            // Add robot velocity so the projectile leads the target
            double actualX = (shootingXSpeed * Math.cos(targetAngle));
            double actualY = (shootingXSpeed * Math.sin(targetAngle)) + (drivetrain.getChassisSpeeds().vyMetersPerSecond);
            targetAngle = Math.atan2(actualY, actualX);
        }
        // Convert from field-relative to robot-relative
        targetAngle -= drivetrain.getState().Pose.getRotation().getRadians();
        // Normalize to [-π, π) so 0 = front of robot
        targetAngle = -Math.IEEEremainder(targetAngle, 2 * Math.PI);
        double smartdashboardangle = Math.toDegrees(targetAngle);
        SmartDashboard.putNumber("turret expected angle", smartdashboardangle);
        return (targetAngle) / (2 * Math.PI);
    }
    /**
     * Kinematics used to figure out the angle
     * Max definetly wrote it trust
    */
    public double maxFormula(){
        targetx = (cachedTarget.getX() - turretPosition.getX()); 
        targety = (cachedTarget.getY() - turretPosition.getY());
        targetDistance = Math.hypot(targetx, targety);
        double v2 = kmaxVelocity * kmaxVelocity;
        double v4 = v2 * v2;
        double d2 = targetDistance * targetDistance;
        double g = 9.80665;
        double h = cachedTargetHeight;
        double discriminant = d2 - (2 * g * targetDistance) * ((h / v2) + ((g * d2) / (2 * v4)));
        if (discriminant >= 0) {
            return 90 - Math.atan2((targetDistance + Math.sqrt(discriminant)), (g * d2 / v2)) / (2 * Math.PI);
        }
        else {
            return 80;
        }
    }
    /**Moves the turret to the wanted spots*/
    public void targetAim(){
        if (robotPose == null || turretPosition == null) return;

        // Compute target once per cycle — used by vectorCalculations() and maxFormula()
        cachedTarget = targetpose();
        cachedTargetHeight = getTargetHeight();
        double desiredHeight = maxFormula();
        desiredRotation  = vectorCalculations();
        // Clamp to soft limits (±135° from front-of-robot zero)
        desiredRotation = MathUtil.clamp(desiredRotation, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        SmartDashboard.putNumber("desiredRotation", desiredRotation);

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
            wasDisabled = false;
        }

        if (turretLocked || !autoAimEnabled) {
            // Explicitly hold current setpoint so stale MotionMagic
            // commands from a previous enable cycle don't persist.
            rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
            heightMotor.setControl(heightoutput.withPosition(heightSetpoint));
            return;
        }

        double rotError = desiredRotation - rotationSetpoint;
        rotationSetpoint += rotationTau * rotError;
        rotationSetpoint = MathUtil.clamp(rotationSetpoint, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        
        // Clamp desired height to soft limits BEFORE computing error so the
        // low-pass filter actually smooths instead of slamming to the limit.
        desiredHeight = MathUtil.clamp(desiredHeight, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        double heightError = desiredHeight - heightSetpoint;
        if (Math.abs(heightError) > .01) {
            heightSetpoint += heightTau * heightError;
        }
        heightSetpoint = MathUtil.clamp(heightSetpoint, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        heightMotor.setControl(heightoutput.withPosition(heightSetpoint));
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
    }

    /**The shoot function makes the robot shoot wow crazy right? never would have expected that */
    public void shoot(){
        flywheelMotor.set(MathUtil.clamp(SmartDashboard.getNumber("Shoot Speed", 1.0), 0, 1));    
        indexerLMotor.setVoltage(-12);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
    }
    /* private double enc2Rad(double EncoderPosition){
        return ((EncoderPosition - 0.5) * 2 * Math.PI);
    } */
    /** Stops shooting */
    public void unshoot(){
        flywheelMotor.stopMotor();
        indexerLMotor.stopMotor();
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    }
    /**Locks the turret */
    public void lockTurret(){
        turretLocked = true;
        //controller.setPID(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);
        //controller2.setPID(RobotConstants.kTurretHeightP, RobotConstants.kTurretHeightI, RobotConstants.kTurretHeightD);
        rotationMotor.setControl(rotationoutput.withPosition(0));
        heightMotor.setControl(heightoutput.withPosition(1.45));
    }
   
    /** Resets the angle of the turret to 0 */
    public void resetTurret(){
        heightMotor.setControl(heightoutput.withPosition(0));
    }
    /**Reverses the direction of the Indexer */
    public void reverseIndexer() {
        indexerLMotor.set(1);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
    }
    /**Starts moving the Indexer */
    public void feedIndexer() {
        indexerLMotor.set(-1);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    /** Nudge the turret manually. Automatically switches to manual mode.
     * Starts slow for precision, ramps up after holding ~1 second.
     * @param direction +1 for right, -1 for left */
    public void manualRotate(double direction) {
        autoAimEnabled = false;
        manualHoldCycles++;
        double t = Math.min(1.0, (double) manualHoldCycles / MANUAL_RAMP_CYCLES);
        double step = MANUAL_STEP_SLOW + (MANUAL_STEP_FAST - MANUAL_STEP_SLOW) * t;
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
        manualHeightHoldCycles++;
        double t = Math.min(1.0, (double) manualHeightHoldCycles / MANUAL_RAMP_CYCLES);
        double step = HEIGHT_STEP_SLOW + (HEIGHT_STEP_FAST - HEIGHT_STEP_SLOW) * t;
        // Start from actual position if setpoint is out of sync
        double base = Math.abs(heightSetpoint - cachedHeightPos) > HEIGHT_MAX_LEAD
                    ? cachedHeightPos : heightSetpoint;
        double desired = base + step * direction;
        desired = MathUtil.clamp(desired, HEIGHT_REVERSE_LIMIT, HEIGHT_FORWARD_LIMIT);
        double lead = desired - cachedHeightPos;
        if (Math.abs(lead) > HEIGHT_MAX_LEAD) {
            desired = cachedHeightPos + Math.copySign(HEIGHT_MAX_LEAD, lead);
        }
        heightSetpoint = desired;
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
    }

    /** Zero the turret rotation and height encoders at current physical position */
    public void zeroTurret() {
        autoAimEnabled = false;
        rotationMotor.setPosition(0);
        rotationSetpoint = 0;
        rotationMotor.setControl(rotationoutput.withPosition(0));
        heightMotor.setPosition(0);
        heightSetpoint = 0;
        heightMotor.setControl(heightoutput.withPosition(0));
    }

    /** @return true if turret is in manual mode */
    public boolean isManualMode() {
        return !autoAimEnabled;
    }

    /**Stops all turret related motors, the Rotation, Height, and Indexer motors */
    public void stopMotors(){
        turretLocked = false;
        rotationMotor.stopMotor();
        heightMotor.stopMotor();
        indexerLMotor.stopMotor();
        indexerRMotor.stopMotor();
        feedMotor.stopMotor();
    }
    @Override
    public void periodic() {
        robotPose = drivetrain.getState().Pose;
        turretPosition = robotPose.transformBy(
            new Transform2d(new Translation2d(RobotConstants.kTurretXOffsetMeters, 0), new Rotation2d()))
            .getTranslation();
        // Only react to the dashboard toggle when the operator actually
        // clicked it (value differs from what we last wrote).
        boolean dashboardAim = SmartDashboard.getBoolean("Auto Aim", autoAimEnabled);
        if (dashboardAim != lastDashboardAim) {
            autoAimEnabled = dashboardAim;
        }

        // Cache CAN reads once per cycle
        cachedRotationPos = rotationMotor.getPosition().getValueAsDouble();
        cachedHeightPos = heightMotor.getPosition().getValueAsDouble();
        cachedFlywheelVel = flywheelMotor.getVelocity().getValueAsDouble();
        cachedThroughborePos = throughbore.get();

        targetAim();

        // Check MCP simulated joystick for height/rotation/shoot commands
        if (mcpJoystick != null && mcpJoystick.isActive() && DriverStation.isEnabled()) {
            int mcpPov = mcpJoystick.getPOV();
            if (mcpPov == 0) manualHeight(1);        // D-pad up
            else if (mcpPov == 180) manualHeight(-1); // D-pad down
            else if (mcpPov == 270) manualRotate(-1); // D-pad left
            else if (mcpPov == 90) manualRotate(1);   // D-pad right

            // Button 1 (A) toggles auto-aim on rising edge
            boolean mcpAPressed = mcpJoystick.getButton(1);
            if (mcpAPressed && !mcpAutoAimWasPressed) {
                if (autoAimEnabled) { autoAimEnabled = false; } else { enableAutoAim(); }
            }
            mcpAutoAimWasPressed = mcpAPressed;

            // Right trigger (axis 3) > 0.5 = shoot
            if (mcpJoystick.getAxis(3) > 0.5) {
                if (!mcpShooting) { shoot(); mcpShooting = true; }
            } else if (mcpShooting) {
                unshoot(); mcpShooting = false;
            }
        } else if (mcpShooting) {
            unshoot(); mcpShooting = false;
        }

        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled);
        lastDashboardAim = autoAimEnabled;
        SmartDashboard.putBoolean("Turret Manual", !autoAimEnabled);
        SmartDashboard.putNumber("Turret Angle", cachedRotationPos * 360.0);
        if (turretPosition != null) {
            turretTargetPub.set(new double[] {
                targetPose != null ? targetPose.getX() : 0,
                targetPose != null ? targetPose.getY() : 0, 0 });
        }

        Logger.recordOutput("Turret/RotationPosition", cachedRotationPos);
        Logger.recordOutput("Turret/HeightPosition", cachedHeightPos);
        Logger.recordOutput("Turret/HeightSetpoint", heightSetpoint);
        Logger.recordOutput("Turret/RotationSetpoint", rotationSetpoint);
        Logger.recordOutput("Turret/FlywheelVelocity", cachedFlywheelVel);
        Logger.recordOutput("Turret/ThroughborePosition", cachedThroughborePos);
        Logger.recordOutput("Turret/TargetAngle", Math.toDegrees(targetAngle));
        Logger.recordOutput("Turret/DesiredMotorRotation", desiredRotation);
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
}