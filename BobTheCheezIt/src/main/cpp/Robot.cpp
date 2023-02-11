// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc2/command/CommandScheduler.h>
#include "hardware/HardwareMap.cpp"
#include "Constants.cpp"
#include <ctre/Phoenix.h>
#include <cmath>
#include "frc/AnalogGyro.h"
#include "frc/I2C.h"


void configMotor(TalonSRX &motor)
{
  motor.ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 1);

  motor.SetInverted(false);
  motor.SetSensorPhase(true);

  motor.ConfigNominalOutputForward(0);
  motor.ConfigNominalOutputReverse(0);
  motor.ConfigPeakOutputForward(1);
  motor.ConfigPeakOutputReverse(-1);

  motor.Config_kF(0, 0.0);
  motor.Config_kP(0, 20);
  motor.Config_kI(0, 0.0);
  motor.Config_kD(0, 1);
  //motor.SetSelectedSensorPosition(0);
  motor.SetInverted(true);
} 

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }

  // Can ID Wheel Locations on the robot
  //        Top
  // Left   0 1   Right
  //        2 3
  //      Bottom

  // 0 - 3 Rotation motors
  // 4 - 7 Drive motors

  configMotor(hw::frontRightRotation);
  configMotor(hw::frontLeftRotation);
  configMotor(hw::backLeftRotation);
  configMotor(hw::backRightRotation);

}

double ClosestAng(double a, double b)
{
  // Returns closest of two angles
  // TODO optimize - full radians
  a = a * (360 / (2 * constants::TIMSCONSTANT));
  b = b * (360 / (2 * constants::TIMSCONSTANT));
  double dir = fmod(b, 360.0) - fmod(a, 360.0);

  if (abs(dir) > 180.0){
    dir = -(copysign(360, dir)) + dir;
  }
  return dir * ((constants::TIMSCONSTANT * 2) / 360);
}

void ClosestDir(double a, double b, double v, double *returnArray){
  double normal = ClosestAng(a, b);
  double flipped = ClosestAng(a, b + constants::TIMSCONSTANT);
  if (abs(normal) <= abs(flipped)){
    returnArray[0] = v;
    returnArray[1] = normal;
  }
  else{
    returnArray[0] = -v;
    returnArray[1] = flipped;
  }
}

double RHO(double x, double y){
  return pow((x * x + y * y), .5);
}

double PHI(double x, double y){
  return atan2(y, x);
}


// Math funcs
/* TODO FUCK
double *CalculateSwerve(double Vx, double Vy, double w)
{
  // Calculates the rotation angles needed for swerve drive
  double a = Vx + w * (constants::ROBOTLENGTH / 2);
  double b = Vy + w * (constants::ROBOTWIDTH / 2);
  double c = Vx - w * (constants::ROBOTLENGTH / 2);
  double d = Vy - w * (constants::ROBOTWIDTH / 2);

  double wheels[8] = {
      Cart2Pol(a, b)[0], (Cart2Pol(a, b)[1] / (2 * 3.1415192)) * 1023,
      Cart2Pol(c, b)[0], (Cart2Pol(c, b)[1] / (2 * 3.1415192)) * 1023,
      Cart2Pol(a, d)[0], (Cart2Pol(a, d)[1] / (2 * 3.1415192)) * 1023,
      Cart2Pol(c, d)[0], (Cart2Pol(c, d)[1] / (2 * 3.1415192)) * 1023};

  return wheels;
}


 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
  // hw::Rotation.Set(ControlMode::PercentOutput, .3);
  //  READ JOYSTICK DATA
  double Vx = hw::rightJoystick.GetX();
  double Vy = -hw::rightJoystick.GetY();
  // right = +, left = -
  double w = 4*hw::rightJoystick.GetTwist();

  if (Vx * Vx < .2){
    Vx = 0;
  }
  if (Vy * Vy < .2){
    Vy = 0;
  }
  if (w * w < .2){
    w = 0;
  }

  //512 = pi
  int rotationOffset = 0;


  //w = -.01 <= w <= .01 ? 0 : w;

  // 0, 2, 4, 6
  // 1, 3, 5, 7
  // 4,5,6,7
  // Calculates the rotation angles needed for swerve drive
  double a = Vx + w * (constants::ROBOTLENGTH / 2);
  double b = Vy + w * (constants::ROBOTWIDTH / 2);
  double c = Vx - w * (constants::ROBOTLENGTH / 2);
  double d = Vy - w * (constants::ROBOTWIDTH / 2);
  
  // working section
  double FLAG = -1;
  if (w == 0){
    FLAG = 1;
  }
  
  /*
  double wheelPowers[8] = {
      RHO(a, b), ((PHI(a, b) / (2 * 3.1415192)) * 1023),
      FLAG*RHO(c, b), ((PHI(c, b) / (2 * 3.1415192)) * 1023),
      RHO(a, d), ((PHI(a, d) / (2 * 3.1415192)) * 1023),
      RHO(c, d), ((PHI(c, d) / (2 * 3.1415192)) * 1023)// + 545
  };
  */


  // left, right, left, right
  double buffer[2]; // power, angle
  double wheelPowers[8];
 // call function with arguments a, b, v
  ClosestDir(((hw::frontLeftRotation.GetSelectedSensorPosition() - 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(a,b),RHO(a, b),buffer);
  wheelPowers[0] = buffer[0];
  wheelPowers[1] = ((hw::frontLeftRotation.GetSelectedSensorPosition() - 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::frontRightRotation.GetSelectedSensorPosition() + 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(c,b),RHO(c, b),buffer);
  wheelPowers[2] = FLAG*buffer[0];
  wheelPowers[3] = ((hw::frontRightRotation.GetSelectedSensorPosition() + 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::backLeftRotation.GetSelectedSensorPosition() + 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(a,d),RHO(a,d),buffer);
  wheelPowers[4] = buffer[0];
  wheelPowers[5] = ((hw::backLeftRotation.GetSelectedSensorPosition() + 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::backRightRotation.GetSelectedSensorPosition() - 256 - 16 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(c,d),RHO(c,d),buffer);
  wheelPowers[6] = buffer[0];
  wheelPowers[7] = (hw::backRightRotation.GetSelectedSensorPosition() - 256 - 16 - 512) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];


  std::cout << wheelPowers[7] << std::endl;
  
  
  // 0x28


  // AB CD - Drive, Rotation
  //std::cout << *wheelPowers << std::endl;

  double A = 0;
  double changeDir = 1;

  if (Vx * Vx + Vy*Vy + w*w != 0){
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[0] + A);
    hw::frontLeftRotation.Set(ControlMode::Position, changeDir * wheelPowers[1] + 128 + 512); // +45

    hw::frontRightDrive.Set(ControlMode::PercentOutput, wheelPowers[2] + A);
    hw::frontRightRotation.Set(ControlMode::Position, changeDir * wheelPowers[3] - 128 + 512); // -45

    hw::backLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[4] + A);
    hw::backLeftRotation.Set(ControlMode::Position, changeDir * wheelPowers[5] - 128 + 512); // -45

    hw::backRightDrive.Set(ControlMode::PercentOutput, wheelPowers[6] + A);
    hw::backRightRotation.Set(ControlMode::Position, changeDir * wheelPowers[7] + 256 + 16 + 512); // +90
  }
  else{
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::frontRightDrive.Set(ControlMode::PercentOutput, 0);
    hw::backLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::backRightDrive.Set(ControlMode::PercentOutput,0);
  }
  /*
  for (int i = 0; i<4; i++){
    hw::controllers[i].Set(ControlMode::Position, WheelPowers[2 * i]);
    if (i == 3){
      hw::D3.Set(ControlMode::PercentOutput, WheelPowers[2 * i + 1]);
      break;
    }
    hw::controllers[i + 4].Set(ControlMode::PercentOutput, WheelPowers[2 * i + 1]);
  }
  */

  /*
  hw::R0.Set(ControlMode::Position, 1024);
  hw::R1.Set(ControlMode::Position, 1024);
  hw::R2.Set(ControlMode::Position, 1024);
  hw::R3.Set(ControlMode::Position, 1024);
  */

  //std::cout << hw::frontRightRotation.GetSelectedSensorPosition() << std::endl;
  //std::cout << hw::backRightRotation.GetSelectedSensorPosition() << std::endl;
  //std::cout << hw::frontLeftRotation.GetSelectedSensorPosition() << std::endl;
  //std::cout << hw::backLeftRotation.GetSelectedSensorPosition() << std::endl;
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
