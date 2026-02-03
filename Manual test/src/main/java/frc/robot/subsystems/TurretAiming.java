package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.RobotConstants;;

public class TurretAiming {
    private Pose2d roboticPose;
    private Pigeon2 turretGyro;
    private Translation2d targetPose;
    private double targetx, targety, targetAngle;
    private TalonFX rotationMotor, heightMotor, flywheelMotor;
    private PIDController controller;

    public void turretingIt() {
        roboticPose = CameraUsing.robotPose2d;
        turretGyro = new Pigeon2(67);
        rotationMotor = new TalonFX(69);
        heightMotor = new TalonFX(41);
        flywheelMotor = new TalonFX(61);
        TalonFXConfiguration turretConfigs = new TalonFXConfiguration();

        turretConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        turretConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        turretConfigs.CurrentLimits.withStatorCurrentLimit(40);
        turretConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        turretConfigs.serialize();
        rotationMotor.getConfigurator().apply(turretConfigs);
        heightMotor.getConfigurator().apply(turretConfigs);
        flywheelMotor.getConfigurator().apply(turretConfigs);

        targetx = (targetpose().getX() - roboticPose.getX()); 
        targety = (targetpose().getY() - roboticPose.getY());
        targetAngle = (Math.atan(targety/targetx));
        double error = turretGyro.getYaw().getValueAsDouble()%360 - targetAngle;
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
    public void targetAim(){
        controller.setPID(RobotConstants.kP, RobotConstants.kI, RobotConstants.kD);
        rotationMotor.set(controller.calculate())
    }
}
