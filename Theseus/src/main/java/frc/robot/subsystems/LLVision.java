// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** @Deprecated We shouldn't be using this */
public class LLVision extends SubsystemBase {

  /**@Deprecated We shouldnt be using this */
  public LLVision() {}

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

  //@Override
  public void periodic() {
    aimAndRange();
  }
}
