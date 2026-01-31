package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;


public class Swerve extends SubsystemBase{
  private final double EncOffset;
  private final AnalogEncoder RotEncoder;
  private final TalonFX DriveMotor;
  private final TalonFX RotMotor;
  private final TalonFXConfiguration rotConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final PIDController rotPID = new PIDController(0, 0, 0);

  public Swerve(int DriveMotorID, int RotMotorID, int RotEncoderID, double AngleOffset){
    DriveMotor = new TalonFX(DriveMotorID);
    RotMotor = new TalonFX(RotMotorID);
    RotEncoder = new AnalogEncoder(RotEncoderID);
    this.EncOffset = AngleOffset;

    rotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotConfig.CurrentLimits.SupplyCurrentLimit = RobotConstants.kCurrentMaxLimit;
    rotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.kRotCurrentStatorLimit;
    RotMotor.getConfigurator().apply(rotConfig);

    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = RobotConstants.kCurrentMaxLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = RobotConstants.kCurrentMaxLimit;
    driveConfig.Slot0.kP = 0.1;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    DriveMotor.getConfigurator().apply(driveConfig);
  }

  private double driveRpS2MpS(double EncoderRotations){
        return ((EncoderRotations * (2 * Math.PI * RobotConstants.kWheelRadius)) / RobotConstants.kDriveGearRatio);
    }

  public double getAbsoluteAngle() {
    double rotations = RotEncoder.get();
    double angle = (rotations * 2 * Math.PI) - Math.toRadians(EncOffset);
    return MathUtil.angleModulus(angle); 
  }

  public void RotPID(double kP, double kI, double kD){
    rotPID.setPID(kP, kI, kD);
  }

  public SwerveModuleState getState(){
    double speed = driveRpS2MpS(DriveMotor.getVelocity().getValueAsDouble());
    double angle = getAbsoluteAngle();
    return new SwerveModuleState(speed, new edu.wpi.first.math.geometry.Rotation2d(angle));
  }
  public SwerveModulePosition getPosition(){
    double distance = driveRpS2MpS(DriveMotor.getPosition().getValueAsDouble());
    double angle = getAbsoluteAngle();
    return new SwerveModulePosition(distance, new Rotation2d(angle));
  }
  public void setDesiredState(SwerveModuleState state){
    double angle = getAbsoluteAngle();
    state.optimize(new Rotation2d(angle));
    state.cosineScale(new Rotation2d(angle));
    double driveOutput = state.speedMetersPerSecond / RobotConstants.kMaxModuleSpeed * RobotConstants.kMaxVoltage;
    double rotOutput = rotPID.calculate(getAbsoluteAngle(), state.angle.getRadians());
    DriveMotor.setVoltage(driveOutput);
    RotMotor.setVoltage(rotOutput);
  }

  public void stopmotors() {
    // yap
    DriveMotor.stopMotor();
    RotMotor.stopMotor();
  }
}