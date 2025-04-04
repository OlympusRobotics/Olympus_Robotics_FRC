// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
    public static final double kFrontLeftEncoderOffset = 0.092;

    public static final int kFrontRightEncoderID = 1;
    public static final double kFrontRightEncoderOffset = 0.371;

    public static final int kBackLeftEncoderID = 2;
    public static final double kBackLeftEncoderOffset = 0.013;

    public static final int kBackRightEncoderID = 3;
    public static final double kBackRightEncoderOffset = 0.813;

    //Gyro ID
    public static final int kGyroID = 9;

    //Rotation kP
    public static final double kFrontLeftP = 0.56;
    public static final double kFrontRightP = 0.505;
    public static final double kBackLeftP = 0.567;
    public static final double kBackRightP = 0.53;

    //Rotation kD
    public static final double kFrontLeftD = 0.0045;
    public static final double kFrontRightD = 0.001;
    public static final double kBackLeftD = 0.0;
    public static final double kBackRightD = 0.0;

    //Module Constants
    public static final double kMaxModuleSpeed = 4.72;
    public static final int kMaxVoltage = 13;
    public static final double kWheelRadius = 0.0508;
    public static final double kGearRatio = 6.75;
    
    //General robot dimension value
    public static final double kDimension = 0.29;
  }
}
