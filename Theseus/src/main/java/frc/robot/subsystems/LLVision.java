// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LLVision extends SubsystemBase {
  //private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  private final CommandSwerveDrivetrain m_drivetrain;
  private final TurretAiming m_turret;
  private Pose2d averagePose2d = new Pose2d(0, 0, new Rotation2d(0));
  public double validtargets, averageTimeStamp;

  public LLVision(CommandSwerveDrivetrain drivetrain, TurretAiming turret) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    validtargets = 0;
  }

  /**
   *  Gets the coordinates and angle of current Apriltag
  */
  public double[] getApriltagCoordinates(){
    return new double[]{
      LimelightHelpers.getTX("limelight-stinky"), 
      LimelightHelpers.getTY("limelight-stinky"), 
      LimelightHelpers.getTA("limelight-stinky")
      };
  }  

  /**
   * Gets the X and Y position of the apriltag being tracked, applies kP and Offsets, and returns a double array for use with swerve drive 
  */
/*   public double[] aimAndRange(){
    double kP = 0.035;

    double angularVel = LimelightHelpers.getTX("limelight-stinky") * kP;
    double rawForwardSpeeds = LimelightHelpers.getTY("limelight-stinky");

    double error = 3.0 - rawForwardSpeeds;

    if (Math.abs(error) < 0.05){
      error = 0.0;
    }

    double forwardSpeeds = error * kP;
    return new double[] {
      forwardSpeeds,
      angularVel
    };


  } */

  /** Estimates the robot's pose using MegaTag1 */
  public void estimateRobotPose(){
    //Updates the limelights position as the turret rotates
    LimelightHelpers.SetRobotOrientation("limelight-still", m_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace("limelight-stinky", -.105 + .21 * Math.cos(m_turret.getCurRotation()), 
    .21 * Math.sin(m_turret.getCurRotation()), .485, 0, 18, m_turret.getCurRotation());
    //updates the still limelights position
    LimelightHelpers.setCameraPose_RobotSpace("limelight-still", -.245, .295, .23, 2, 10, 90);
    //gets the MegaTag1 pose estimation from the limelight
    LimelightHelpers.PoseEstimate LL4Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-stinky");
    LimelightHelpers.PoseEstimate LL2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-still");
      //System.out.println(LL2Pose.avgTagDist);
      if (!LimelightHelpers.getTV("limelight-still")) return; //if neither limelight sees a tag, return null
      
      
      System.out.println(LL2Pose.pose);
      //m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.01,.01,999999999));
      //m_drivetrain.addVisionMeasurement(LL2Pose.pose, LL2Pose.timestampSeconds);
      if (LL2Pose.pose.getX() != 0) {
        /* averagePose2d = new Pose2d(( LL2Pose.pose.getX()), (LL2Pose.pose.getY()),
        new Rotation2d((LL2Pose.pose.getRotation().getRadians()))); */
        //m_drivetrain.addVisionMeasurement(LL2Pose.pose, LL2Pose.timestampSeconds);
      }
      validtargets = 0;
      averageTimeStamp = 0;

      
  }

  //@Override
  public void periodic() {
    //estimateRobotPose();
  }
}
