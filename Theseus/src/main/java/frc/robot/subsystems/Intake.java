package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Intake extends SubsystemBase {

  private final TalonFX m_inkMot, intFWMot, m_inkMotFollower;
  private double target;
  public Intake() {

    // defining + configuring motors
    m_inkMot = new TalonFX(RobotConstants.kIntakeID);
    m_inkMotFollower = new TalonFX(RobotConstants.kIntakeFollowerID);
    intFWMot = new TalonFX(RobotConstants.kIntakeFWID);

    target = 0;

    m_inkMot.setPosition(0);
    m_inkMot.getConfigurator().apply(intakeConf);
    m_inkMotFollower.setControl(new Follower(m_inkMot.getDeviceID(), MotorAlignmentValue.Opposed));
    intFWMot.getConfigurator().apply(intakeFWConf);
  }

  // turns indexer on and sends intake out
  public void startIntake() {
    target = ActivatedPos;
    intFWMot.set(1);
  }
  
  // stops indexer and returns intake to original position
  public void endIntake() {
    target = 0;
    intFWMot.set(0);
  }
  public void outakeIntake() {
    target = ActivatedPos;
    intFWMot.set(-1);
  }
  @Override
  public void periodic() {
    m_inkMot.setControl(new MotionMagicVoltage(target));
  }
}