#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include <cameraserver/CameraServer.h>

HardwareMap hw; //allows you to access the different hardware components
bool cMode = false; //drifting - called cMode for cameron's mode

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  hw.bigArmSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  //hw.liftSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}


void mainPeriodic(){}
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

// Called every 20 ms when enabled on teleop
void Robot::TeleopPeriodic() {
  
  // START OF DRIVING
  //forward = +, backward = -
  double joystickY = hw.rightJoystick.GetY();

  double sens = .3; //limits turning speed

  //right = +, left = -
  double joystickX = sens * hw.rightJoystick.GetX();
  
  if (joystickX * joystickX < .02 ) joystickX = 0;

  //if you didn't already know you can twist the joysticks
  //right = +, left = -
  double joystickTwist = hw.rightJoystick.GetTwist();


  if (abs(joystickX) < 0.1) joystickX = 0.0; //turning deadzone

  double leftMotorPower = joystickY + joystickX;
  //clipping so it's betweeen -1 and 1
  leftMotorPower = leftMotorPower > 1 ? 1 : leftMotorPower;
  leftMotorPower = leftMotorPower < -1 ? -1 : leftMotorPower;
  
  //right motors are negative since they face the opposite direction
  double rightMotorPower = -joystickY + joystickX;
  //clipping
  rightMotorPower = rightMotorPower > 1 ? 1 : rightMotorPower;
  rightMotorPower = rightMotorPower < -1 ? -1 : rightMotorPower;
  
  //motor turning
  hw.frontLeftMotor.Set(ControlMode::PercentOutput, leftMotorPower);
  hw.backLeftMotor.Set(ControlMode::PercentOutput, leftMotorPower);
  hw.frontRightMotor.Set(ControlMode::PercentOutput, rightMotorPower);
  hw.backRightMotor.Set(ControlMode::PercentOutput, rightMotorPower);

  //drifting - it's the button on the right on the face of the joysticks
  if(hw.leftJoystick.GetRawButtonReleased(4)) cMode = !cMode;

  //drift right
  if(joystickTwist > 0 && cMode){
    hw.frontLeftMotor.Set(ControlMode::PercentOutput, 1);
    hw.backLeftMotor.Set(ControlMode::PercentOutput, 1);
  }
  //drift left
  else if(joystickTwist < 0 && cMode){
    hw.frontRightMotor.Set(ControlMode::PercentOutput, -1);
    hw.backRightMotor.Set(ControlMode::PercentOutput, -1);
  }

  // END OF DRIVE
  // Power for the winches
  double bigArmPower = hw.xBox.GetLeftY();
  hw.leftArmMotor.Set(ControlMode::PercentOutput, bigArmPower);
  hw.rightArmMotor.Set(ControlMode::PercentOutput, bigArmPower);

  // Left bumper toggles big arm
  if(hw.xBox.GetLeftBumperPressed()) hw.bigArmSolenoid.Toggle();

  //if (hw.xB){



}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
