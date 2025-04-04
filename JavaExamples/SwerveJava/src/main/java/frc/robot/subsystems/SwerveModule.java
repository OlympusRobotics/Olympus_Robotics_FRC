package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.RobotConstants;

public class SwerveModule extends SubsystemBase {
    private final TalonFX m_driveMotor;
    private final SparkMax m_rotationMotor;
    private final AnalogEncoder m_rotationEncoder;

    private final PIDController m_rotationPIDController = new PIDController(0, 0, 0);

    private double enc2Distance(double EncoderPosition){
        return ((EncoderPosition / RobotConstants.kGearRatio) * (2 * Math.PI * RobotConstants.kGearRatio));
    }

    private double rps2mps(double EncoderRotations){
        return ((EncoderRotations * (2 * Math.PI * RobotConstants.kWheelRadius)) / RobotConstants.kGearRatio);
    }

    private double enc2Rad(double EncoderPosition){
        return ((EncoderPosition - 0.5) * 2 * Math.PI);
    }
    
    public SwerveModule(
        int DriveMotorID,
        int RotationMotorID,
        int RotationEncoderID,
        double RotationEncoderOffset) {
        m_driveMotor = new TalonFX(DriveMotorID);
        m_rotationMotor = new SparkMax(RotationMotorID, MotorType.kBrushless);
        m_rotationEncoder = new AnalogEncoder(RotationEncoderID, 0, RotationEncoderOffset);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        driveConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        driveConfig.CurrentLimits.withStatorCurrentLimit(40);
        driveConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        driveConfig.serialize();

        SparkMaxConfig rotationConfig = new SparkMaxConfig();
        
        rotationConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        m_driveMotor.getConfigurator().apply(driveConfig);
        m_rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);        
    }

    public void setPIDValues(double kP, double kI, double kD) { 
        m_rotationPIDController.setPID(kP, kI, kD);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            rps2mps(m_driveMotor.getVelocity().getValueAsDouble()),
            new Rotation2d(enc2Rad(m_rotationEncoder.get()))
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            enc2Distance(m_driveMotor.getPosition().getValueAsDouble()),
            new Rotation2d(enc2Rad(m_rotationEncoder.get()))
        );
    }

    public void setState(SwerveModuleState newState){
        newState.optimize(new Rotation2d(enc2Rad(m_rotationEncoder.get())));
        newState.cosineScale(new Rotation2d(enc2Rad(m_rotationEncoder.get())));

        double rotationOutput = m_rotationPIDController.calculate(enc2Rad(m_rotationEncoder.get()), newState.angle.getRadians());

        m_driveMotor.setVoltage((newState.speedMetersPerSecond / RobotConstants.kMaxModuleSpeed) / RobotConstants.kMaxVoltage);
        m_rotationMotor.set(rotationOutput);

    }

    public void stopDrive(){
        m_driveMotor.stopMotor();
        m_rotationMotor.stopMotor();
    }
}
