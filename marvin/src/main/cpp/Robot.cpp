// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "hardware/HardwareMap.cpp"
#include "ctre/phoenix/motorcontrol/ControlMode.h"

HardwareMap hw;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
}

void mainPeriodic(){
  
  //forward/backward on joysticks
  double joystickY = hw.leftJoystick.GetY() + hw.rightJoystick.GetY();
  //clipped to be between -1 and 1
  joystickY = joystickY > 1 ? 1 : joystickY;
  joystickY = joystickY < -1 ? -1 : joystickY;
  
  double joystickX = hw.leftJoystick.GetX() + hw.rightJoystick.GetX();
  //clipped to be between -1 and 1
  joystickX = joystickX > 1 ? 1 : joystickX;
  joystickX = joystickX < -1 ? -1 : joystickX;
  
  
  hw.frontLeftMotor.Set(ControlMode::PercentOutput, -joystickY);
  hw.backLeftMotor.Set(ControlMode::PercentOutput, -joystickY);
  hw.frontRightMotor.Set(ControlMode::PercentOutput, joystickY);
  hw.backRightMotor.Set(ControlMode::PercentOutput, joystickY);
  
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
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {mainPeriodic();}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
