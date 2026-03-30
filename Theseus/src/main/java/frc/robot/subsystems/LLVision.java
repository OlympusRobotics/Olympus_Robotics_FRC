// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LLVision extends SubsystemBase {
  //private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  private final CommandSwerveDrivetrain m_drivetrain;
  private final TurretAiming m_turret;

  public LLVision(CommandSwerveDrivetrain drivetrain, TurretAiming turret) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
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
  public double[] aimAndRange(){
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


  }

  /** Estimates the robot's pose using MegaTag1 */
  public void estimateRobotPose(){
    //TODO these need updated, the offsets, especially the turret rotation may need to be corrected
    //Updates the limelights position as the turret rotates
    LimelightHelpers.setCameraPose_RobotSpace("limelight-stinky", 0, 0, 1.5, 30, 0, m_turret.getCurRotation() + 0);
    //gets the MegaTag1 pose estimation from the limelight
     LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-stinky");
      
      if(mt1.tagCount == 0) return;
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if(mt1.rawFiducials[0].ambiguity > .7) return;
        if(mt1.rawFiducials[0].distToCamera > 3) return;
      }
      //Applies the estimated pose
      m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      m_drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
  }

  //@Override
  public void periodic() {
    aimAndRange();
  }
}
