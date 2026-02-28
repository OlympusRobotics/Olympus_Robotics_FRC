package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
  public static class OperatorConstants {
    //Controller Ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class RobotConstants {
    //Swerve Drive Motor IDs
    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontLeftRotationID = 2;

    public static final int kFrontRightDriveID = 3;
    public static final int kFrontRightRotationID = 4;

    public static final int kBackLeftDriveID = 5;
    public static final int kBackLeftRotationID = 6;

    public static final int kBackRightDriveID = 7;
    public static final int kBackRightRotationID = 8;

    //Swerve Drive Rotation Encoder ports and offsets
    public static final int kFrontLeftEncoderID = 0;
    public static final double kFrontLeftEncoderOffset = 0.542;

    public static final int kFrontRightEncoderID = 1;
    public static final double kFrontRightEncoderOffset = 0.206;

    public static final int kBackLeftEncoderID = 2;
    public static final double kBackLeftEncoderOffset = 0.443;

    public static final int kBackRightEncoderID = 3;
    public static final double kBackRightEncoderOffset = 0.565;

    //Gyro ID
    public static final int kGyroID = 9;

    //Climber ID
    public static final int kClimberID = 10;
    
    //Turret device ids
    //In parentheses because im too lazy
    public static final int kTurretRotationID = (13);
    public static final int kTurretHeightID = (14);
    public static final int kTurretFlywheelID = (15);
    public static final int kTurretIndexerID = (19);
    public static final int kTurretFeedID = (20);

    //intake device ids
    public static final int kIntakeID = 16;
    public static final int kIntakeFollowerID = 17;
    public static final int kIntakeFWID = 18;

    //Rotation kP
    public static final double kFrontLeftP = 0.56;
    public static final double kFrontRightP = 0.505;
    public static final double kBackLeftP = 0.567;
    public static final double kBackRightP = 0.53;

    //Rotation kI
    public static final double kFrontLeftI = 0.0;
    public static final double kFrontRightI = 0.0;
    public static final double kBackLeftI = 0.0;
    public static final double kBackRightI = 0.0;

    //Rotation kD
    public static final double kFrontLeftD = 0.0045;
    public static final double kFrontRightD = 0.001;
    public static final double kBackLeftD = 0.0;
    public static final double kBackRightD = 0.0;

    //auto PID1
    public static final double kUnoP = 0.2;
    public static final double kUnoI = 0.2;
    public static final double kUnoD = 0.2;

    //auto PID2
    public static final double kDosP = 0.2;
    public static final double kDosI = 0.2;
    public static final double kDosD = 0.2;

    //Module Constants
    public static final double kMaxModuleSpeed = 4.72;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final int kMaxVoltage = 12;
    public static final double kWheelRadius = 0.0508;
    public static final double kDriveGearRatio = 6.75;
    public static final double kRotGearRatio = 12.8;
    public static final double kCurrentMaxLimit = 40;
    public static final double kRotCurrentStatorLimit = 30;
    
    //Robot Dimensions
    public static final double kDistancetoLeftSwerveCenter = 0.22225;
    public static final double kDistanceToRightSwerveCenter = -0.22225;
    public static final double kDistanceToFrontSwerveCenter = 0.26035;
    public static final double kDistanceToBackSwerveCenter = -0.26035;
    public static final double kTrackWidth = 0.6858;
    public static final double kRobotlength = 0.6604;
    public static final double kCameraHeight = .2;
    
    //Turret aiming pid
    public static final double kTurretRotationP = 0.6; //kP ≈ 0.4–0.8 
    public static final double kTurretRotationI = 0;
    public static final double kTurretRotationD = 0.005; // kD ≈ 0.001–0.01
    public static final double kTurretRotationVelocity = 180; //rps
    public static final double kTurretRotationAcceleration = 200; //rps²

    public static final double kTurretHeightP = 2; //kP slightly higher than yaw
    public static final double kTurretHeightI = 0;
    public static final double kTurretHeightD = 0.0025; //kD small
    public static final double kTurretHeightVelocity = 1000; //rps
    public static final double kTurretHeightAcceleration = 1000; //rps²
      
    //Intake PID
    public static final double kIntakeP = 1; //bullcrap values
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0.001;
    public static final double kIntakevelocity = 100;
    public static final double kIntakeAcceleration = 100;

    //robot config

  }
  //Config CLasses
  public static class TurretConfigs {
    /** Configuration for the thingy that angles the shooter from {@link TurretConfigs}*/
    public static final TalonFXConfiguration heightConfigs = new TalonFXConfiguration();
    /** Configuration for the motor that rotates the turret from {@link TurretConfigs} */
    public static final TalonFXConfiguration rotationConfigs = new TalonFXConfiguration();
    /** Configuration for the flywheel from {@link TurretConfigs} */
    public static final TalonFXConfiguration flyConfigs = new TalonFXConfiguration();
    /** Configuration for the indexer from {@link TurretConfigs} */
    public static final TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    public static final Double rotationRatio = 100.0;
    public static final Double turretHeight = .508; 
    public static final Double kmaxVelocity = 4.71;
    public static final Double heightRatio = 5.0;
    public static final Double smoothRotation = 0.0;
    public static final Double smoothHeight = 0.0;
    public static final Double rotationTao = .05;
    public static final Double heightTao = .1;
    static {
      //This applies all the configuration
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
        rotationConfigs.serialize();

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

        
        //flywheel stuff
        flyConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        flyConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        flyConfigs.CurrentLimits.withStatorCurrentLimit(40);
        flyConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        flyConfigs.serialize(); //save

        //indexer stuff
        indexerConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        indexerConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        indexerConfigs.CurrentLimits.withStatorCurrentLimit(40);
        indexerConfigs.CurrentLimits.withStatorCurrentLimitEnable(true);
        indexerConfigs.serialize(); //save
    }
  }
  public static class IntakeConstants {
    /** Configs for the intake motors from {@link IntakeConstants} */
    public static final TalonFXConfiguration intakeConf = new TalonFXConfiguration();
    /** Configs for the intakeFW motor from {@link IntakeConstants} */
    public static final TalonFXConfiguration intakeFWConf = new TalonFXConfiguration();
    public static final double ActivatedPos = 1.45;
    static {
      //
      intakeConf.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
      intakeConf.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
      intakeConf.CurrentLimits.withStatorCurrentLimit(40);
      intakeConf.CurrentLimits.withStatorCurrentLimitEnable(true);
      //intakeOneConf.Feedback.SensorToMechanismRatio = 6.7;
      intakeConf.Slot0.kP = RobotConstants.kIntakeP;
      intakeConf.Slot0.kI = RobotConstants.kIntakeI;
      intakeConf.Slot0.kD = RobotConstants.kIntakeD;
      intakeConf.Slot0.kG = 0.3;
      intakeConf.MotionMagic.MotionMagicCruiseVelocity = RobotConstants.kIntakevelocity;
      intakeConf.MotionMagic.MotionMagicAcceleration = RobotConstants.kIntakeAcceleration;
      intakeConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      intakeConf.serialize();

      //Intake FW config
      intakeFWConf.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
      intakeFWConf.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
      intakeFWConf.CurrentLimits.withStatorCurrentLimit(40);
      intakeFWConf.CurrentLimits.withStatorCurrentLimitEnable(true);
      intakeFWConf.serialize();
    }
    
  }
}