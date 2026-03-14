package frc.robot.subsystems;
// NOTE: Changes to motors, CAN IDs, or aiming logic must be reflected in Theseus/README.md (Turret Aiming section).

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.TurretConfigs.*;

import org.littletonrobotics.junction.Logger;

public class TurretAiming extends SubsystemBase {
    private Pose2d roboticPose;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, 
    smoothRotation, smoothHeight, rotationTao, heightTao, desiredAngle;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, indexerLMotor, indexerRMotor, feedMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleArrayPublisher turretTargetPub;
    private final PIDController stinkyPIDcontrollerthatmayormaynotwork;
    private final DutyCycleEncoder throughbore;
    private boolean turretLocked = false;
    private boolean wasDisabled = true;
    private boolean manualMode = false;
    private static final double MANUAL_STEP = 0.005; // turret rotations per cycle (~90°/sec)

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
        roboticPose = new Pose2d();
        kmaxVelocity = 2;
        heightTao = .15;
        rotationTao = .15;
        desiredAngle = 0;
        turretTargetPub = NetworkTableInstance.getDefault()
            .getTable("Pose").getDoubleArrayTopic("turretTarget").publish();
        //Set up motors
        rotationMotor.getConfigurator().apply(rotationConfigs);
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply
        indexerLMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        stinkyPIDcontrollerthatmayormaynotwork = new PIDController(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);

        SmartDashboard.putData("Zero Turret", new InstantCommand(() -> rotationMotor.setPosition(0)).ignoringDisable(true));
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
            if (roboticPose.getX() > 12.5) {
                return targetPose = new Translation2d(11.914, 4.035);
            }
            //not red alliance hub area
            else if (roboticPose.getX() < 12.5) {
                //upper portion of field
                if (roboticPose.getY() > 4) {
                    return targetPose = new Translation2d(15, 7);
                }
                //lower portion of field
                else if (roboticPose.getY() < 4) {
                    return targetPose = new Translation2d(15, 1);
                }
            }
        }
        //if on blue alliance
        else if (!CommandSwerveDrivetrain.getAlliance()) {
            //blue alliance hub area
            if (roboticPose.getX() < 4) {
                return targetPose = new Translation2d(4.621, 4.035);
            } 
            //not blue alliance hub area
            else if (roboticPose.getX() > 4) {
                //upper portion of field
                if (roboticPose.getY() > 4) {
                    return targetPose = new Translation2d(2, 7);
                }
                //lower portion of field
                else if (roboticPose.getY() < 4) {
                    return targetPose = new Translation2d(2, 1);
                }
            }
        }
        return targetPose;
    }

    /**Gets the height of the target based on which target it is aiming at, and subtracts the robot height to get the actual height */
    public double getTargetHeight(){
        if (roboticPose.getX() < 4 || roboticPose.getX() > 12.5) {
            return 1.8288 - turretHeight;
        }
        else {
            return turretHeight;
        }
    }

    /**Calculating the actual angle while the robot is moving*/
    public double vectorCalculations() {
        Translation2d target = targetpose();
        targetx = (target.getX() - roboticPose.getX()); 
        targety = (target.getY() - roboticPose.getY());
        targetAngle = (Math.atan2(targety, targetx));
        double shootingXSpeed = kmaxVelocity*Math.cos(maxFormula());
        // Add robot velocity so the projectile leads the target
        double actualX = (shootingXSpeed * Math.cos(targetAngle));
        double actualY = (shootingXSpeed * Math.sin(targetAngle)) + (drivetrain.getChassisSpeeds().vyMetersPerSecond);
        targetAngle = Math.atan2(actualY, actualX);
        // Convert from field-relative to robot-relative
        targetAngle -= drivetrain.getState().Pose.getRotation().getRadians();
        targetAngle += Math.PI;
        // Normalize to [0, 2π)
        targetAngle = Math.IEEEremainder(targetAngle, 2 * Math.PI);
        if (targetAngle < 0) { targetAngle += 2 * Math.PI; }
        double smartdashboardangle = Math.toDegrees(targetAngle);
        SmartDashboard.putNumber("turret expected angle", smartdashboardangle);
        return (targetAngle) / (2 * Math.PI) / (2 * Math.PI);
    }
    /**
     * Kinematics used to figure out the angle
     * Max definetly wrote it trust
    */
    public double maxFormula(){
        Translation2d target = targetpose();
        targetx = (target.getX() - roboticPose.getX()); 
        targety = (target.getY() - roboticPose.getY());
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
        if (roboticPose == null) return;

        // Don't accumulate error while disabled — it causes the turret to
        // wind up and slam on first enable.
        if (!DriverStation.isEnabled()) {
            wasDisabled = true;
            return;
        }

        // On the first enabled cycle, sync the smoothing state to the
        // actual motor positions so there's no accumulated jump.
        if (wasDisabled) {
            smoothRotation = rotationMotor.getPosition().getValueAsDouble();
            smoothHeight = heightMotor.getPosition().getValueAsDouble();
            wasDisabled = false;
        }

        targetpose();
        getTargetHeight();
        double desiredHeight = maxFormula();
        desiredAngle  = vectorCalculations();
        SmartDashboard.putNumber("desiredangle", desiredAngle);

        if (turretLocked) return;

        double rotError = desiredAngle - rotationMotor.getPosition().getValueAsDouble();
        
        smoothRotation += rotationTao * rotError;
        
        double heightError = desiredHeight - smoothHeight;
        if (Math.abs(heightError) > .01) {
            smoothHeight += heightTao * heightError;
        }
        heightMotor.setControl(heightoutput.withPosition(smoothHeight));
        rotationMotor.setControl(rotationoutput.withPosition(smoothRotation));
        //rotationMotor.set(stinkyPIDcontrollerthatmayormaynotwork.calculate(throughbore.get(), smoothRotation));
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
     * @param direction +1 for right, -1 for left */
    public void manualRotate(double direction) {
        manualMode = true;
        smoothRotation += MANUAL_STEP * direction;
        rotationMotor.setControl(rotationoutput.withPosition(smoothRotation));
    }

    /** Switch back to auto-aim mode */
    public void enableAutoAim() {
        manualMode = false;
    }

    /** Zero the turret rotation encoder at current physical position */
    public void zeroTurret() {
        manualMode = true;
        rotationMotor.setPosition(0);
        smoothRotation = 0;
    }

    /** @return true if turret is in manual mode */
    public boolean isManualMode() {
        return manualMode;
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
        roboticPose = drivetrain.getState().Pose;
        if (!manualMode) {
            targetAim();
        }
        SmartDashboard.putBoolean("Turret Manual", manualMode);
        SmartDashboard.putNumber("targetAngle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("pose?", targety);
        SmartDashboard.putNumber("pose2", targetx);
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("smoothRotation", smoothRotation);
        SmartDashboard.putNumber("Turret Angle", rotationMotor.getPosition().getValueAsDouble() * 360.0);
        if (targetPose != null) {
            turretTargetPub.set(new double[] { targetPose.getX(), targetPose.getY(), 0 });
        }

        Logger.recordOutput("Turret/RotationPosition", rotationMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Turret/HeightPosition", heightMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Turret/FlywheelVelocity", flywheelMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/ThroughborePosition", throughbore.get());
        Logger.recordOutput("Turret/TargetAngle", Math.toDegrees(targetAngle));
        Logger.recordOutput("Turret/DesiredAngle", desiredAngle);
        Logger.recordOutput("Turret/SmoothRotation", smoothRotation);
    }
}