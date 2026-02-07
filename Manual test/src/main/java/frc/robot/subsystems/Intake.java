package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

  private final TalonFX m_inkMot, intFWMot;
  private final double notActivatedPos, ActivatedPos, gearRatio;
  private final PIDController controller;

  public Intake() {

    // defining + configuring motors
    m_inkMot = new TalonFX(11);
    intFWMot = new TalonFX(12);
    TalonFXConfiguration intakeOneConf = new TalonFXConfiguration();
    controller = new PIDController(0, 0, 0);
    ActivatedPos = 6.7;
    gearRatio = 6.7;
    notActivatedPos = 0;

    intakeOneConf.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeOneConf.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    intakeOneConf.CurrentLimits.withStatorCurrentLimit(40);
    intakeOneConf.CurrentLimits.withStatorCurrentLimitEnable(true);
    intakeOneConf.serialize();
    m_inkMot.getConfigurator().apply(intakeOneConf);
  }

  // turns indexer on and sends intake out
  public void startIntake() {
    controller.setPID(RobotConstants.kIntakeP, RobotConstants.kIntakeI, RobotConstants.kIntakeD);
    m_inkMot.set(controller.calculate(m_inkMot.get(), ActivatedPos) / gearRatio);
    intFWMot.set(1);
  }
  
  // stops indexer and returns intake to original position
  public void endIntake() {
    controller.setPID(RobotConstants.kIntakeP, RobotConstants.kIntakeI, RobotConstants.kIntakeD);
    m_inkMot.set(controller.calculate(m_inkMot.get(), notActivatedPos) / gearRatio);
    intFWMot.set(0);
  }
  public void outakeIntake() {
    controller.setPID(RobotConstants.kIntakeP, RobotConstants.kIntakeI, RobotConstants.kIntakeD);
    m_inkMot.set(controller.calculate(m_inkMot.get(), ActivatedPos) / gearRatio);
    intFWMot.set(-1);
  }
}