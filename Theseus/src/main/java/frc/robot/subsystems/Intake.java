package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

  private final TalonFX m_inkMot, intFWMot, m_inkMotFollower;
  private final double ActivatedPos;
  private double target;
  private final TalonFXConfiguration intakeOneConf, intakeFWConf;
  public Intake() {

    // defining + configuring motors
    m_inkMot = new TalonFX(16);
    m_inkMotFollower = new TalonFX(17);
    intFWMot = new TalonFX(18);
    intakeOneConf = new TalonFXConfiguration();
    intakeFWConf = new TalonFXConfiguration();
    ActivatedPos = 1.45;
    target = 0;
    m_inkMot.setPosition(0);

    intakeOneConf.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    intakeOneConf.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeOneConf.CurrentLimits.withStatorCurrentLimit(40);
    intakeOneConf.CurrentLimits.withStatorCurrentLimitEnable(true);
    //intakeOneConf.Feedback.SensorToMechanismRatio = 6.7;
    intakeOneConf.Slot0.kP = RobotConstants.kIntakeP;
    intakeOneConf.Slot0.kI = RobotConstants.kIntakeI;
    intakeOneConf.Slot0.kD = RobotConstants.kIntakeD;
    intakeOneConf.Slot0.kG = 0.3;
    intakeOneConf.MotionMagic.MotionMagicCruiseVelocity = RobotConstants.kIntakevelocity;
    intakeOneConf.MotionMagic.MotionMagicAcceleration = RobotConstants.kIntakeAcceleration;
    intakeOneConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    intakeOneConf.serialize();
    m_inkMot.getConfigurator().apply(intakeOneConf);
    m_inkMotFollower.setControl(new Follower(m_inkMot.getDeviceID(), MotorAlignmentValue.Opposed));

    intakeFWConf.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    intakeFWConf.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    intakeFWConf.CurrentLimits.withStatorCurrentLimit(40);
    intakeFWConf.CurrentLimits.withStatorCurrentLimitEnable(true);
    intakeFWConf.serialize();
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