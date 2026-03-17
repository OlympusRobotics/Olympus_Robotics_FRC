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
import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.TurretConfigs.*;

import org.littletonrobotics.junction.Logger;

public class TurretAiming extends SubsystemBase {
    private Pose2d robotPose;
    private Translation2d turretPosition;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, 
    rotationSetpoint, heightSetpoint, rotationTau, heightTau, desiredRotation;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, indexerLMotor, indexerRMotor, feedMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleArrayPublisher turretTargetPub;
    //private final PIDController stinkyPIDcontrollerthatmayormaynotwork;
    private final DutyCycleEncoder throughbore;
    private boolean turretLocked = false;
    private boolean wasDisabled = true;
    private boolean autoAimEnabled = false;
    private boolean lastDashboardAim = false;
    private int manualHoldCycles = 0;
    private static final double MANUAL_STEP_SLOW = 0.002; // fine step for short presses
    private static final double MANUAL_STEP_FAST = 0.008; // fast step after holding ~1s
    private static final int MANUAL_RAMP_CYCLES = 50;     // cycles before ramping up (~1s at 50Hz)

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

        SmartDashboard.putData("Zero Turret", new InstantCommand(() -> rotationMotor.setPosition(0)).ignoringDisable(true));
        SmartDashboard.putBoolean("Velocity Compensation", false);
        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled);
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
        Translation2d target = targetpose();
        targetx = (target.getX() - turretPosition.getX()); 
        targety = (target.getY() - turretPosition.getY());
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
        Translation2d target = targetpose();
        targetx = (target.getX() - turretPosition.getX()); 
        targety = (target.getY() - turretPosition.getY());
        targetDistance = (Math.sqrt(Math.pow(targetx, 2)+Math.pow(targety, 2)));
        if ((Math.pow(targetDistance, 2) - (2*9.80665*targetDistance) * 
        ((getTargetHeight() / (Math.pow(kmaxVelocity, 2))) + ((9.80665 * Math.pow(targetDistance, 2))/(2 * 
        Math.pow(kmaxVelocity, 4))))) >= 0) {
            return 90 - Math.atan2((targetDistance + Math.sqrt(Math.pow(targetDistance, 2) - (2*9.80665*targetDistance) * 
            ((getTargetHeight() / (Math.pow(kmaxVelocity, 2))) + ((9.80665 * Math.pow(targetDistance, 2))/(2 * 
            Math.pow(kmaxVelocity, 4)))))), (9.80665 * Math.pow(targetDistance, 2) / (Math.pow(kmaxVelocity, 2)))) / (2 * Math.PI);
        }
        else {
            return 80;
        }
    }
    /**Moves the turret to the wanted spots*/
    public void targetAim(){
        if (robotPose == null || turretPosition == null) return;

        // Always compute the target so DesiredPose is available even while disabled.
        targetpose();
        getTargetHeight();
        double desiredHeight = maxFormula();
        desiredRotation  = vectorCalculations();
        // Wrap desiredRotation into soft limits by adding/subtracting a full rotation
        while (desiredRotation < ROTATION_REVERSE_LIMIT) { desiredRotation += 1.0; }
        while (desiredRotation > ROTATION_FORWARD_LIMIT) { desiredRotation -= 1.0; }
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
            rotationSetpoint = rotationMotor.getPosition().getValueAsDouble();
            heightSetpoint = heightMotor.getPosition().getValueAsDouble();
            wasDisabled = false;
        }

        if (turretLocked || !autoAimEnabled) return;

        double rotError = desiredRotation - rotationSetpoint;
        rotationSetpoint += rotationTau * rotError;
        rotationSetpoint = MathUtil.clamp(rotationSetpoint, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        
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
        flywheelMotor.set(1);    
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
        rotationSetpoint += step * direction;
        rotationSetpoint = MathUtil.clamp(rotationSetpoint, ROTATION_REVERSE_LIMIT, ROTATION_FORWARD_LIMIT);
        rotationMotor.setControl(rotationoutput.withPosition(rotationSetpoint));
    }

    /** Resets the manual rotation ramp counter (call when D-pad is released) */
    public void resetManualRamp() {
        manualHoldCycles = 0;
    }

    /** Switch back to auto-aim mode */
    public void enableAutoAim() {
        autoAimEnabled = true;
    }

    /** Zero the turret rotation encoder at current physical position */
    public void zeroTurret() {
        autoAimEnabled = false;
        rotationMotor.setPosition(0);
        rotationSetpoint = 0;
        rotationMotor.setControl(rotationoutput.withPosition(0));
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
        targetAim();
        SmartDashboard.putBoolean("Auto Aim", autoAimEnabled);
        lastDashboardAim = autoAimEnabled;
        SmartDashboard.putBoolean("Turret Manual", !autoAimEnabled);
        SmartDashboard.putNumber("targetAngle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("pose?", targety);
        SmartDashboard.putNumber("pose2", targetx);
        SmartDashboard.putNumber("desiredRotation", desiredRotation);
        SmartDashboard.putNumber("rotationSetpoint", rotationSetpoint);
        SmartDashboard.putNumber("Turret Angle", rotationMotor.getPosition().getValueAsDouble() * 360.0);
        if (turretPosition != null) {
            SmartDashboard.putNumber("Turret X", turretPosition.getX());
            SmartDashboard.putNumber("Turret Y", turretPosition.getY());
            double turretOffsetDistance = turretPosition.getDistance(robotPose.getTranslation());
            SmartDashboard.putNumber("Turret Offset Distance", turretOffsetDistance);
            SmartDashboard.putNumber("Turret Offset Expected", Math.abs(RobotConstants.kTurretXOffsetMeters));
        }
        if (targetPose != null) {
            turretTargetPub.set(new double[] { targetPose.getX(), targetPose.getY(), 0 });
        }

        Logger.recordOutput("Turret/RotationPosition", rotationMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Turret/HeightPosition", heightMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Turret/FlywheelVelocity", flywheelMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/ThroughborePosition", throughbore.get());
        Logger.recordOutput("Turret/TargetAngle", Math.toDegrees(targetAngle));
        Logger.recordOutput("Turret/DesiredMotorRotation", desiredRotation);
        Logger.recordOutput("Turret/RotationSetpoint", rotationSetpoint);
        if (turretPosition != null) {
            Logger.recordOutput("Turret/PositionX", turretPosition.getX());
            Logger.recordOutput("Turret/PositionY", turretPosition.getY());
            double turretOffsetDistance = turretPosition.getDistance(robotPose.getTranslation());
            Logger.recordOutput("Turret/OffsetDistance", turretOffsetDistance);
            Logger.recordOutput("Turret/OffsetExpected", Math.abs(RobotConstants.kTurretXOffsetMeters));
        }

        // Pose2d for turret: desired and actual field headings.
        // targetAngle is robot-relative (negated), so the field-relative desired heading is:
        //   robotHeading - targetAngle
        // For actual, convert motor position (mechanism rotations) back to radians
        // via motorPos * 2π, then add robot heading for field-relative angle.
        if (robotPose != null && turretPosition != null) {
            double robotHeading = robotPose.getRotation().getRadians();

            double actualMotorPos = rotationMotor.getPosition().getValueAsDouble();
            double actualFieldAngle = robotHeading - actualMotorPos * 2 * Math.PI;
            Logger.recordOutput("Turret/ActualPose", new Pose2d(turretPosition, new Rotation2d(actualFieldAngle)));

            double desiredFieldAngle = robotHeading - targetAngle;
            Logger.recordOutput("Turret/DesiredPose", new Pose2d(turretPosition, new Rotation2d(desiredFieldAngle)));
        }
    }
}