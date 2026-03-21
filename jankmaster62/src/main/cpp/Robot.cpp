// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>
#include <iostream>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "frc_sub/robot/hardware/HardwareMap.cpp"

#include "ctre/phoenix/motorcontrol/ControlMode.h"

bool cMode = false;
bool pToggled = false;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  /*if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }*/
  mainPeriodic();
}

void Robot::TeleopInit() {
  fmt::print("TeleopInit\n");
  std::cout << "test" << std::endl;
}

void Robot::TeleopPeriodic() {mainPeriodic();}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {mainPeriodic();}

void mainPeriodic() {
  HardwareMap HardwareMap;

  //This is the motor in the front that picks up the balls
  if(HardwareMap.xBox.GetRightTriggerAxis() > 0.1) HardwareMap.intakeMotor.Set(ControlMode::PercentOutput, .4);
  else if(HardwareMap.xBox.GetRightBumper()) HardwareMap.intakeMotor.Set(ControlMode::PercentOutput, -.4);
  else HardwareMap.intakeMotor.Set(ControlMode::PercentOutput, 0.0);

  //This is that belt thing that brings the balls to the shooting wheel
  if(HardwareMap.xBox.GetLeftTriggerAxis() > 0.1) HardwareMap.beltMotor.Set(ControlMode::PercentOutput, 1.0);
  else if(HardwareMap.xBox.GetLeftBumper()) HardwareMap.beltMotor.Set(ControlMode::PercentOutput, -1.0);
  else HardwareMap.beltMotor.Set(ControlMode::PercentOutput, 0.0);

  //These are for the shooter wheels
  if(HardwareMap.xBox.GetAButton()){
    HardwareMap.outputMotor1.Set(ControlMode::PercentOutput, 1.0);
    HardwareMap.outputMotor2.Set(ControlMode::PercentOutput, 1.0);
  }
  else if(HardwareMap.xBox.GetBButton()){
    HardwareMap.outputMotor1.Set(ControlMode::PercentOutput, -1.0);
    HardwareMap.outputMotor2.Set(ControlMode::PercentOutput, -1.0);
  }
  else {
    HardwareMap.outputMotor1.Set(ControlMode::PercentOutput, 0.0);
    HardwareMap.outputMotor2.Set(ControlMode::PercentOutput, 0.0);
  }

  //Drift mode toggle
  if(HardwareMap.rightJoystick.GetRawButton(4)) cMode = !cMode;

  if(HardwareMap.rightJoystick.GetTwist() < 0 && cMode) {
    HardwareMap.frontRightMotor.Set(ControlMode::PercentOutput, 1.0);
    HardwareMap.frontLeftMotor.Set(ControlMode::PercentOutput, 1.0);
  }
  else if(HardwareMap.rightJoystick.GetTwist() > 0 && cMode) {
    HardwareMap.frontRightMotor.Set(ControlMode::PercentOutput, -1.0);
    HardwareMap.frontLeftMotor.Set(ControlMode::PercentOutput, -1.0);
  }
  else {
    HardwareMap.frontRightMotor.Set(ControlMode::PercentOutput, 0.0);
    HardwareMap.frontLeftMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  if (HardwareMap.xBox.GetYButton()){
    pToggled = !pToggled;
    sleep(100);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
