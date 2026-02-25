package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.RobotConstants;

public class TurretAiming extends SubsystemBase {
    private Pose2d roboticPose;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity, heightRatio, rotationRatio, 
    smoothRotation, smoothHeight, rotationTao, heightTao;
    private TalonFX rotationMotor, heightMotor, flywheelMotor, indexerMotor, feedMotor;
    private TalonFXConfiguration rotationConfigs, heightConfigs, flyConfigs, indexerConfigs;
    private MotionMagicVoltage rotationoutput, heightoutput;
    private CommandSwerveDrivetrain drivetrain;

    public TurretAiming(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotationMotor = new TalonFX(13);
        heightMotor = new TalonFX(14);
        flywheelMotor = new TalonFX(15);
        indexerMotor = new TalonFX(19);
        feedMotor = new TalonFX(20);
        rotationConfigs = new TalonFXConfiguration();
        heightConfigs = new TalonFXConfiguration();
        flyConfigs = new TalonFXConfiguration();
        indexerConfigs = new TalonFXConfiguration();
        turretHeight = .508; 
        kmaxVelocity = 4.71;
        rotationoutput = new MotionMagicVoltage(0);
        heightoutput = new MotionMagicVoltage(0);
        heightRatio = 5;
        rotationRatio = 100;
        smoothRotation = 0;
        smoothHeight = 0;
        rotationTao = .05;
        heightTao = .1;

        //basic motor configurations
        rotationConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        rotationConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        //current limits
        rotationConfigs.CurrentLimits.withStatorCurrentLimit(40);
        rotationConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        //motor limits
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 130.0/360.0;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -220.0/360.0;
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotationConfigs.Feedback.SensorToMechanismRatio = rotationRatio;
        rotationConfigs.Slot0.kP = RobotConstants.kTurretRotationP;
        rotationConfigs.Slot0.kI = RobotConstants.kTurretRotationI;
        rotationConfigs.Slot0.kD = RobotConstants.kTurretRotationD;
        rotationConfigs.MotionMagic.MotionMagicCruiseVelocity = RobotConstants.kTurretRotationVelocity;
        rotationConfigs.MotionMagic.MotionMagicAcceleration = RobotConstants.kTurretRotationAcceleration;
        //save
        rotationConfigs.serialize();
        //apply
        rotationMotor.getConfigurator().apply(rotationConfigs);

        //height motors stuff
        heightConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        heightConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        heightConfigs.CurrentLimits.withStatorCurrentLimit(40);
        heightConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        heightConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1.5;
        heightConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        heightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        heightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        heightConfigs.Feedback.SensorToMechanismRatio = heightRatio;
        heightConfigs.Slot0.kP = RobotConstants.kTurretHeightP;
        heightConfigs.Slot0.kI = RobotConstants.kTurretHeightI;
        heightConfigs.Slot0.kD = RobotConstants.kTurretHeightD;
        heightConfigs.MotionMagic.MotionMagicCruiseVelocity = RobotConstants.kTurretRotationVelocity;
        heightConfigs.MotionMagic.MotionMagicAcceleration = RobotConstants.kTurretHeightAcceleration;
        heightMotor.getConfigurator().apply(heightConfigs); //apply to the motor

        //flywheel stuff
        flyConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        flyConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        flyConfigs.CurrentLimits.withStatorCurrentLimit(40);
        flyConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        flyConfigs.serialize(); //save
        flywheelMotor.getConfigurator().apply(flyConfigs); //apply

        indexerConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        indexerConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        indexerConfigs.serialize(); //save
        indexerMotor.getConfigurator().apply(indexerConfigs);
        feedMotor.setControl(new Follower(indexerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }
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

    //gets the height of the target based on which target it is aiming at, and subtracts the robot height to get the actual height
    public double getTargetHeight(){
        if (roboticPose.getX() < 4 || roboticPose.getX() > 12.5) {
            return 1.8288 - turretHeight;
        }
        else {
            return turretHeight;
        }
    }

    //calculating the actual angle while the robot is moving
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

    //moves the turret to the wanted spots
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

    //kinematics used to figure out the angle
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
    public void shoot(){
        flywheelMotor.set(1);    
        indexerMotor.set(-1);
        feedMotor.set(1);
        
    }
    public void unshoot(){
        flywheelMotor.stopMotor();
        feedMotor.stopMotor();
    }
    public void lockTurret(){
        //controller.setPID(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);
        //controller2.setPID(RobotConstants.kTurretHeightP, RobotConstants.kTurretHeightI, RobotConstants.kTurretHeightD);
        rotationMotor.setControl(rotationoutput.withPosition(0));
        heightMotor.setControl(heightoutput.withPosition(1.45));
    }
   
    public void resetTurret(){
        heightMotor.setControl(heightoutput.withPosition(0));
    }
    public void reverseIndexer() {
        indexerMotor.set(-1);
    }
    public void feedIndexer() {
        indexerMotor.set(1);
    }
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