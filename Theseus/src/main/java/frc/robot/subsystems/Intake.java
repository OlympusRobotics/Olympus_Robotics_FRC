package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// NOTE: Changes to motors, CAN IDs, or positions must be reflected in Theseus/README.md (Intake section).

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public final TalonFX m_inkMot;
  private final TalonFX intFWMot;
  private final TalonFX intFWMotFollower;
  private final TalonFX m_inkMotFollower;
  private final TalonFX vibratorMot;
  private double target;
  public Intake() {

    // defining + configuring motors
    m_inkMot = new TalonFX(RobotConstants.kIntakeID);
    m_inkMotFollower = new TalonFX(RobotConstants.kIntakeFollowerID);
    intFWMot = new TalonFX(RobotConstants.kIntakeFWID);
    intFWMotFollower = new TalonFX(RobotConstants.kIntakeFWFollowerID);
    vibratorMot = new TalonFX(RobotConstants.kIntakeVibratorID); //this motor was removed from the robot as it did not help as much as we wanted it to

    target = 0;

    m_inkMot.getConfigurator().apply(intakeConf);
    m_inkMotFollower.setControl(new Follower(m_inkMot.getDeviceID(), MotorAlignmentValue.Opposed));
    intFWMot.getConfigurator().apply(intakeFWConf);
    intFWMotFollower.getConfigurator().apply(intakeFWConf);
    intFWMotFollower.setControl(new Follower(intFWMot.getDeviceID(), MotorAlignmentValue.Opposed));
    vibratorMot.getConfigurator().apply(intakeFWConf);
  }

  //spins the intake flywheels to pull in fuel
  public void startIntake() {
    m_inkMot.set(1); //max power, 1 is max 0 is minimum, -1 is max in opposite direction
    m_inkMotFollower.set(1);
  }
  public void jerkIntake() { //this lifted and lowered the intake in hopes of getting fuel into the indexer until the shafts that lifted the intake rounded out in which case we stopped using this
    m_inkMot.set(-1);
    m_inkMotFollower.set(1);
    intFWMot.set(.4);
    vibratorMot.set(.35);
    intFWMotFollower.set(-.4);

  }
  public void spinflywheel() { //this command was before configurations were properly applied to the motors, did the same thing as startIntake
    intFWMot.set(.4);
    vibratorMot.set(.35);
    intFWMotFollower.set(-.4);


  }
  public void stopspin() { //stops motors relating to the fylwheels
    intFWMot.stopMotor();;
    vibratorMot.stopMotor();
    intFWMotFollower.stopMotor();

  }
  // stops indexer and returns intake to original position
  public void endIntake() {
    m_inkMot.setVoltage(0); //setvoltage is similar to set, 12 is max, 0 is minimum, -12 is max in opposite direction
    m_inkMotFollower.setControl(new Follower(m_inkMot.getDeviceID(), MotorAlignmentValue.Opposed)); //this just made the motor do the same thing as the "master motor" but in the opposite direction
    m_inkMotFollower.set(0);
  }
  public void outakeIntake() { //I have just noticed that code was never pushed after second competition but this command was brought back to full power and worked beautifully
    intFWMot.set(-.4);
    vibratorMot.set(.35);
    intFWMotFollower.set(.4);
  }
  public void zeroPosition() {
    m_inkMot.setPosition(0);
    m_inkMotFollower.setPosition(0);
    target = 0;
  }
  @Override
  public void periodic() {
    Logger.recordOutput("Intake/TargetPosition", target);
    Logger.recordOutput("Intake/ActualPosition", m_inkMot.getPosition().getValueAsDouble());
    Logger.recordOutput("Intake/FlywheelOutput", intFWMot.get());
    Logger.recordOutput("Intake/FlywheelFollowerOutput", intFWMotFollower.get());
  }
}