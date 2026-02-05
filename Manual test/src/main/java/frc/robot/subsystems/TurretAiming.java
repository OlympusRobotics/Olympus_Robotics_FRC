package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.RobotConstants;;

public class TurretAiming {
    private Pose2d roboticPose;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle, turretHeight, targetDistance, kmaxVelocity;
    private TalonFX rotationMotor, heightMotor, flywheelMotor;
    private PIDController controller, controller2;
    private TalonFXConfiguration rotationConfigs, heightConfigs;

    public void turretingIt() {
        roboticPose = CameraUsing.robotPose2d;
        rotationMotor = new TalonFX(69);
        heightMotor = new TalonFX(41);
        flywheelMotor = new TalonFX(420);
        rotationConfigs = new TalonFXConfiguration();
        heightConfigs = new TalonFXConfiguration();
        turretHeight = .508; 
        kmaxVelocity = 4.71;

        //basic motor configurations
        rotationConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        rotationConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        //current limits
        rotationConfigs.CurrentLimits.withStatorCurrentLimit(40);
        rotationConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        //motor limits
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2000/2 - 10;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 2000/2 - 10;
        rotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        //save
        rotationConfigs.serialize();
        //apply
        rotationMotor.getConfigurator().apply(rotationConfigs);

        heightConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        heightConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        heightConfigs.CurrentLimits.withStatorCurrentLimit(40);
        heightConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        heightConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2000/360 * 90;
        heightConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 2000/360 * 90;
        heightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        heightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        heightMotor.getConfigurator().apply(heightConfigs);
        
        targetx = (targetpose().getX() - roboticPose.getX()); 
        targety = (targetpose().getY() - roboticPose.getY());
        targetAngle = (Math.atan(targety/targetx));
        targetDistance = (Math.sqrt(Math.pow(targetx, 2)+Math.pow(targety, 2)));
    }
    public Translation2d targetpose() {
        targetPose = new Translation2d(0, 0);
        //if on red alliance
        if (Drivetrain.getAlliance()) {
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
        else if (!Drivetrain.getAlliance()) {
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
        double chassisSpeeded = Math.pow((Math.pow(Drivetrain.getChassisSpeeds().vxMetersPerSecond, 2) + 
        Math.pow(Drivetrain.getChassisSpeeds().vyMetersPerSecond, 2)), .5);
        double chassisAngle = Math.atan(Drivetrain.getChassisSpeeds().vyMetersPerSecond/Drivetrain.getChassisSpeeds().vyMetersPerSecond);
        double shootingXSpeed = kmaxVelocity*Math.cos(maxFormula());
        double actualX = (shootingXSpeed * Math.sin(targetAngle)) - (chassisSpeeded * Math.sin(chassisAngle));
        double actualY = (shootingXSpeed * Math.cos(targetAngle)) - (chassisSpeeded * Math.cos(chassisAngle));
        targetAngle = Math.atan(actualY/actualX);
        return 0;
    }

    //moves the turret to the wanted spots
    public void targetAim(){
        controller.setPID(RobotConstants.kTurretRotationP, RobotConstants.kTurretRotationI, RobotConstants.kTurretRotationD);
        controller2.setPID(RobotConstants.kTurretHeightP, RobotConstants.kTurretHeightI, RobotConstants.kTurretHeightD);
        rotationMotor.set(controller.calculate(rotationMotor.get(), (2000/360) * vectorCalculations()));
        heightMotor.set(controller2.calculate(heightMotor.get(), (2000/360) * maxFormula()));
    }

    //kinematics used to figure out the angle
    public double maxFormula(){
        return Math.atan((targetDistance + Math.sqrt(Math.pow(targetDistance, 2) - (2*9.80665*targetDistance) * 
        ((getTargetHeight() / (Math.pow(kmaxVelocity, 2))) + ((9.80665 * Math.pow(targetDistance, 2))/(2 * Math.pow(kmaxVelocity, 4)))))) / 
        (9.80665 * Math.pow(targetDistance, 2) / (Math.pow(kmaxVelocity, 2))));
    }
    public void shoot(){
        flywheelMotor.set(kmaxVelocity);
    }
}