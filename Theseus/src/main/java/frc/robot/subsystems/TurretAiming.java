package frc.robot.subsystems;
// NOTE: Changes to motors, CAN IDs, or aiming logic must be reflected in Theseus/README.md (Turret Aiming section).

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.TurretConfigs.*;

public class TurretAiming extends SubsystemBase {
    private Pose2d roboticPose;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, 
    smoothRotation, smoothHeight, rotationTao, heightTao, desiredAngle;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, indexerLMotor, indexerRMotor, feedMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController stinkyPIDcontrollerthatmayormaynotwork;
    private final DutyCycleEncoder throughbore;

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
        kmaxVelocity = 25;
        heightTao = .05;
        rotationTao = .05;
        desiredAngle = 0;
        //Set up motors
        rotationMotor.getConfigurator().apply(rotationConfigs);
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply
        indexerLMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        indexerRMotor.setControl(new Follower(indexerLMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        stinkyPIDcontrollerthatmayormaynotwork = new PIDController(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);
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
                return targetPose = new Translation2d(4.621, 4.035);
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
                return targetPose = new Translation2d(11.914, 4.035);
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
        double actualX = (shootingXSpeed * Math.cos(targetAngle)) - (drivetrain.getChassisSpeeds().vxMetersPerSecond);
        double actualY = (shootingXSpeed * Math.sin(targetAngle)) - (drivetrain.getChassisSpeeds().vyMetersPerSecond);
        targetAngle = Math.atan2(actualY, actualX);
        targetAngle -= drivetrain.getState().Pose.getRotation().getRadians();
        if (targetAngle > Math.toRadians(180)) {targetAngle -= Math.toRadians(360);}
        if (targetAngle < Math.toRadians(-180)) {targetAngle += Math.toRadians(360);}
        double smartdashboardangle = Math.toDegrees(targetAngle / 2 * Math.PI);
        SmartDashboard.putNumber("turret expected angle", smartdashboardangle);
        return targetAngle / (2 * Math.PI);
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
        targetpose();
        getTargetHeight();
        double desiredHeight = maxFormula();
        desiredAngle  = vectorCalculations();
        double rotError = desiredAngle - smoothRotation;
        
        smoothRotation += rotationTao * rotError;
        
        double heightError = desiredHeight - smoothHeight;
        if (Math.abs(heightError) > .01) {
            smoothHeight += heightTao * heightError;
        }
        heightMotor.setControl(heightoutput.withPosition(smoothHeight));
        rotationMotor.set(stinkyPIDcontrollerthatmayormaynotwork.calculate(throughbore.get(), smoothRotation));
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
    /**Stops all turret related motors, the Rotation, Height, and Indexer motors */
    public void stopMotors(){
        rotationMotor.stopMotor();
        heightMotor.stopMotor();
        indexerLMotor.stopMotor();
        indexerRMotor.stopMotor();
        feedMotor.stopMotor();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("pose?", targety);
        SmartDashboard.putNumber("pose2", targetx);
        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("smoothRotation", smoothRotation);
        roboticPose = drivetrain.getState().Pose;
        //targetAim(false);
    }
}