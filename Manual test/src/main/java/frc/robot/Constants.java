package frc.robot;

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
    public static final int kMaxVoltage = 13;
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
    public static final double kTurretRotationP = 0;
    public static final double kTurretRotationI = 0;
    public static final double kTurretRotationD = 0;
    public static final double kTurretRotationVelocity = 0;
    public static final double kTurretRotationAcceleration = 0;

    public static final double kTurretHeightP = 0;
    public static final double kTurretHeightI = 0;
    public static final double kTurretHeightD = 0;
    public static final double kTurretHeightVelocity = 0;
    public static final double kTurretHeightAcceleration = 0;
      
    //Intake PID
    public static final double kIntakeP = 0;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;

  }
}