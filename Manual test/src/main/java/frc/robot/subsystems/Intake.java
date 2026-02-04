package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {

  private final TalonFX m_inkMot, m_indexer;

  // setting positions
  final double notActivatedPos = 0.0;
  final double ActivatedPos = 6.9;

  // setting speed
  final double gearRatio = 6.7;
  final double wheelCircumference = 67;
  final double speed = (gearRatio * 4.5) / wheelCircumference;

  public Intake() {

    // defining + configuring motors
    m_inkMot = new TalonFX(11);
    m_indexer = new TalonFX(12);
    TalonFXConfiguration intakeOneConf = new TalonFXConfiguration();
    TalonFXConfiguration indexerConf = new TalonFXConfiguration();


    intakeOneConf.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    intakeOneConf.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    intakeOneConf.CurrentLimits.withStatorCurrentLimit(40);
    intakeOneConf.CurrentLimits.withStatorCurrentLimitEnable(true);
    intakeOneConf.serialize();
    m_inkMot.getConfigurator().apply(intakeOneConf);

    indexerConf.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    indexerConf.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    indexerConf.CurrentLimits.withStatorCurrentLimit(40);
    indexerConf.CurrentLimits.withStatorCurrentLimitEnable(true);
    indexerConf.serialize();
    m_indexer.getConfigurator().apply(indexerConf);

    
  }
  // turns indexer on and sends intake out
  public void startIntake() {
    m_inkMot.setPosition(ActivatedPos);
    m_indexer.set(speed);
  }
  
  // stops indexer and returns intake to original position
  public void endIntake() {
    m_indexer.stopMotor();
    m_inkMot.setPosition(notActivatedPos);
  }

  // gets the motor bc idk
  public TalonFX getTalon() {return m_inkMot;}
}