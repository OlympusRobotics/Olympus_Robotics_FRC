package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    smoothRotation, smoothHeight, rotationTao, heightTao;
    private final TalonFX rotationMotor, heightMotor, flywheelMotor, indexerMotor, feedMotor;
    private final MotionMagicVoltage rotationoutput, heightoutput;
    private final CommandSwerveDrivetrain drivetrain;

    /** Subsystem for the turret */
    public TurretAiming(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotationMotor = new TalonFX(RobotConstants.kTurretRotationID);
        heightMotor =   new TalonFX(RobotConstants.kTurretHeightID);
        flywheelMotor = new TalonFX(RobotConstants.kTurretFlywheelID);
        indexerMotor =  new TalonFX(RobotConstants.kTurretIndexerID);
        feedMotor =     new TalonFX(RobotConstants.kTurretFeedID);
        rotationoutput =new MotionMagicVoltage(0);
        heightoutput =  new MotionMagicVoltage(0);

        //Set up motors
        rotationMotor.getConfigurator().apply(rotationConfigs);
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply
        indexerMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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
        double chassisSpeeded = Math.pow((Math.pow(drivetrain.getChassisSpeeds().vxMetersPerSecond, 2) + 
        Math.pow(drivetrain.getChassisSpeeds().vyMetersPerSecond, 2)), .5);
        double chassisAngle = Math.atan2(drivetrain.getChassisSpeeds().vyMetersPerSecond, drivetrain.getChassisSpeeds().vxMetersPerSecond);
        double shootingXSpeed = kmaxVelocity*Math.cos(maxFormula());
        double actualX = (shootingXSpeed * Math.sin(targetAngle)) - (chassisSpeeded * Math.sin(chassisAngle));
        double actualY = (shootingXSpeed * Math.cos(targetAngle)) - (chassisSpeeded * Math.cos(chassisAngle));
        targetAngle = Math.atan2(actualY, actualX);
        if (targetAngle > Math.toRadians(135)) {targetAngle -= Math.toRadians(360);}
        if (targetAngle < Math.toRadians(-225)) {targetAngle += Math.toRadians(360);}
        double smartdashboardangle = Math.toDegrees(targetAngle);
        SmartDashboard.putNumber("turret expected angle", smartdashboardangle);
        return targetAngle / (2 * Math.PI);
    }

    /**Moves the turret to the wanted spots*/
    public void targetAim(){
        if (roboticPose == null) return;
        targetpose();
        getTargetHeight();
        double desiredAngle  = vectorCalculations();
        double desiredHeight = maxFormula();
        double rotError = desiredAngle - smoothRotation;
        if (Math.abs(rotError) > .02) {
            smoothRotation += rotationTao * rotError;
        }
        double heightError = desiredHeight - smoothHeight;
        if (Math.abs(heightError) > .01) {
            smoothHeight += heightTao * heightError;
        }
        rotationMotor.setControl(rotationoutput.withPosition(smoothRotation));
        heightMotor.setControl(heightoutput.withPosition(smoothHeight));
    }

    /**
     * Kinematics used to figure out the angle
     * Max definetly wrote it trust
    */
    public double maxFormula(){
        targetx = (targetpose().getX() - roboticPose.getX()); 
        targety = (targetpose().getY() - roboticPose.getY());
        targetAngle = (Math.atan2(targety, targetx));
        targetDistance = (Math.sqrt(Math.pow(targetx, 2)+Math.pow(targety, 2)));
        if ((Math.pow(targetDistance, 2) - (2*9.80665*targetDistance) * 
        ((getTargetHeight() / (Math.pow(kmaxVelocity, 2))) + ((9.80665 * Math.pow(targetDistance, 2))/(2 * 
        Math.pow(kmaxVelocity, 4))))) >= 0) {
            return Math.atan2((targetDistance + Math.sqrt(Math.pow(targetDistance, 2) - (2*9.80665*targetDistance) * 
            ((getTargetHeight() / (Math.pow(kmaxVelocity, 2))) + ((9.80665 * Math.pow(targetDistance, 2))/(2 * 
            Math.pow(kmaxVelocity, 4)))))), (9.80665 * Math.pow(targetDistance, 2) / (Math.pow(kmaxVelocity, 2)))) / (2 * Math.PI);
        }
        else {
            return 0;
        }
    }
    /**The shoot function makes the robot shoot wow crazy right? never would have expected that */
    public void shoot(){
        flywheelMotor.set(1);    
        indexerMotor.set(-1);
        feedMotor.set(1);
        
    }
    /** Stops shooting */
    public void unshoot(){
        flywheelMotor.stopMotor();
        feedMotor.stopMotor();
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
        indexerMotor.set(-1);
    }
    /**Starts moving the Indexer */
    public void feedIndexer() {
        indexerMotor.set(1);
    }
    /**Stops all turret related motors, the Rotation, Height, and Indexer motors */
    public void stopMotors(){
        rotationMotor.set(0);
        heightMotor.set(0);
        indexerMotor.set(0);
    }
    @Override
    public void periodic() {
        if (CameraUsing.robotPose2d == null) {return;}
        roboticPose = CameraUsing.robotPose2d;
    }
}