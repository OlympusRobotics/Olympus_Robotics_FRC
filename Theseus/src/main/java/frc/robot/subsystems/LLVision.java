// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public Pose2d estimateRobotPose(){
    //Updates the limelights position as the turret rotates
    LimelightHelpers.setCameraPose_RobotSpace("limelight-stinky", -.105 + .21 * Math.cos(m_turret.getCurRotation()), 
    .21 * Math.sin(m_turret.getCurRotation()), .485, 0, 18, m_turret.getCurRotation());
    //updates the stationary limelights position
    LimelightHelpers.setCameraPose_RobotSpace("limelight-stationary", -.245, .295, .23, 0, 10, 90);
    //gets the MegaTag1 pose estimation from the limelight
    LimelightHelpers.PoseEstimate LL4Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-stinky");
    LimelightHelpers.PoseEstimate LL2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-stationary");
      
      if (LL4Pose.tagCount == 0 && LL2Pose.tagCount == 0) return null; //if neither limelight sees a tag, return null
      if ((LL4Pose.tagCount >= 1 && LL4Pose.rawFiducials.length >= 1) || (LL2Pose.tagCount >= 1 && LL2Pose.rawFiducials.length >= 1)) {
        if (LL4Pose.rawFiducials[0].ambiguity > .7 && LL2Pose.rawFiducials[0].ambiguity > .7) return null; //if both limelights have high ambiguity, return null
        if (LL4Pose.rawFiducials[0].distToCamera > 3 && LL2Pose.rawFiducials[0].distToCamera > 3) return null; //if all visible targets are far away, return null
        if (LL4Pose.tagCount >= 1 && LL4Pose.rawFiducials.length >= 1) {
          validtargets += 1;
          averageTimeStamp += LL4Pose.timestampSeconds;
        }
        if (LL2Pose.tagCount >= 1 && LL2Pose.rawFiducials.length >= 1) {
          validtargets += 1;
          averageTimeStamp += LL2Pose.timestampSeconds;
        }
      };
      
      //combine the poses
      averageTimeStamp /= validtargets;
      averagePose2d = new Pose2d((LL4Pose.pose.getX() + LL2Pose.pose.getX()) / validtargets, (LL4Pose.pose.getY() + LL2Pose.pose.getY()) / validtargets,
      new Rotation2d((LL4Pose.pose.getRotation().getRadians() + LL2Pose.pose.getRotation().getRadians()) / validtargets));
      System.out.println("timestamp: " + averageTimeStamp);
      System.out.println("Posex: " + averagePose2d.getX());
      validtargets = 0;
      averageTimeStamp = 0;
      return averagePose2d;
      
  }

  //@Override
  public void periodic() {
    m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,10));
    m_drivetrain.addVisionMeasurement(estimateRobotPose(), averageTimeStamp);
  }
}
