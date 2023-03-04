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
#include <vector>


// useful variables
double starting_angle = 0;
double seconds = 3;
std::vector<double> averageVector;
frc::AnalogInput pins{0};
frc::DigitalInput armLimit {0};
frc::DoubleSolenoid claw{frc::PneumaticsModuleType::CTREPCM, 0, 2};

rev::CANSparkMax armHeight1{12, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax armHeight2{13, rev::CANSparkMax::MotorType::kBrushed};
VictorSPX armExtension = {16};
//rev::CANSparkMax armExtension{14, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax clawMotor0{14, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax clawMotor1{15, rev::CANSparkMax::MotorType::kBrushed};


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
  motor.Config_kD(0, 2);
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
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  double drivePower = .5;
  double wheelPowers[8] = {drivePower,0,drivePower,0, drivePower,0,drivePower,0};
  double dogcalibration[4] = {1,.69420,1,.69420};
  if (seconds > 0 ){
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[0] * dogcalibration[0]);
    hw::frontLeftRotation.Set(ControlMode::Position, wheelPowers[1] + 128 + 512); // +45

    hw::frontRightDrive.Set(ControlMode::PercentOutput, wheelPowers[2] * dogcalibration[1]);
    hw::frontRightRotation.Set(ControlMode::Position,  wheelPowers[3] - 128 + 512); // -45

    hw::backLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[4] * dogcalibration[2]);
    hw::backLeftRotation.Set(ControlMode::Position, wheelPowers[5] - 128 + 512); // -45

    hw::backRightDrive.Set(ControlMode::PercentOutput, wheelPowers[6] * dogcalibration[3]);
    hw::backRightRotation.Set(ControlMode::Position,  wheelPowers[7] + 256 + 16 + 512); // +90

    seconds -= .02;
  }
  else {
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::frontLeftRotation.Set(ControlMode::Position, wheelPowers[1] + 128 + 512); // +45

    hw::frontRightDrive.Set(ControlMode::PercentOutput, 0);
    hw::frontRightRotation.Set(ControlMode::Position,  wheelPowers[3] - 128 + 512); // -45

    hw::backLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::backLeftRotation.Set(ControlMode::Position, wheelPowers[5] - 128 + 512); // -45

    hw::backRightDrive.Set(ControlMode::PercentOutput, 0);
    hw::backRightRotation.Set(ControlMode::Position,  wheelPowers[7] + 256 + 16 + 512); // +90

  }
}

void Robot::TeleopInit() {
  armLimitPos = 0;
  switchHit = false;
  goingToPos = 0;

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }

  // Can ID Wheel Locations on the robot
  //          Top
  // Left   0 1   Right
  //          2 3
  //        Bottom

  // 0 - 3 Rotation motors
  // 4 - 7 Drive motors
  pins.SetOversampleBits(0);
  pins.SetAverageBits(10);

  starting_angle = ((double)pins.GetValue() / constants::GYROMAX) * 2 * constants::TIMSCONSTANT + constants::TIMSCONSTANT/2;
  //for(int i = 0; i < constants::slidingPWMWindowSize; i++) averageVector.push_back(starting_angle);

  claw.Set(frc::DoubleSolenoid::Value::kReverse);
  configMotor(hw::frontRightRotation);
  configMotor(hw::frontLeftRotation);
  configMotor(hw::backLeftRotation);
  configMotor(hw::backRightRotation);
}

double ClosestAng(double a, double b) {
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

void ClosestDir(double a, double b, double v, double *returnArray) {
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

double RHO(double x, double y) {
  return pow((x * x + y * y), .5);
}

double PHI(double x, double y) {
  return atan2(y, x);
}

// Math funcs

double oldError = 0;
double setAngle = 0;
bool set = 0;
void Robot::TeleopPeriodic() {

  // hw::Rotation.Set(ControlMode::PercentOutput, .3);
  //  READ JOYSTICK DATA
  double gyro_radians = (pins.GetAverageValue() / constants::GYROMAX) * 2 * constants::TIMSCONSTANT + constants::TIMSCONSTANT / 2 - starting_angle;
  double Vx = 1.5 * hw::rightJoystick.GetX();
  double Vy = - 1.5 * hw::rightJoystick.GetY();

  // field oriented swerve
  double temp = Vy * cos(gyro_radians) + Vx * sin(gyro_radians);
  Vx = -Vy * sin(gyro_radians) + Vx * cos(gyro_radians);
  Vy = temp;
  // right = +, left = -
  double w = 3 * hw::rightJoystick.GetTwist();

  
  if (hw::rightJoystick.GetRawButton(1)){
    setAngle = 0;
    double pos_gyro_rad = gyro_radians;
    
    
    if (pos_gyro_rad >= constants::TIMSCONSTANT && pos_gyro_rad <= 2 * constants::TIMSCONSTANT){
      pos_gyro_rad -= 2 * constants::TIMSCONSTANT;
    }
    

    double error = setAngle - pos_gyro_rad;


    
    if (error*error > 0){
      w = 4 * error;// - 300 * (oldError - error)/20;
      
      oldError = error;

    }
  }
  else if (hw::rightJoystick.GetRawButton(2)){
    setAngle = constants::TIMSCONSTANT;
    double pos_gyro_rad = gyro_radians;

    double error = setAngle - pos_gyro_rad;
    
    if (error*error > 0){
      w = 4 * error;// - 300 * (oldError - error)/20;
      
      oldError = error;

    }
  }

  // Deadzones
  /*
  if (Vx * Vx < .001){
    Vx = 0;
  }
  if (Vy * Vy < .001){
    Vy = 0;
  }
  if (w * w < .001){
    w = 0;
  }
  */

  //512 = pi
  int rotationOffset = 0;


  //w = -.01 <= w <= .01 ? 0 : w;

  // 0, 2, 4, 6
  // 1, 3, 5, 7
  // 4,5,6,7
  // Calculates the rotation angles needed for swerve drive
  double a = Vx - w * (constants::ROBOTLENGTH / 2);
  double b = Vx + w * (constants::ROBOTLENGTH / 2);
  double c = Vy - w * (constants::ROBOTWIDTH / 2);
  double d = Vy + w * (constants::ROBOTWIDTH / 2);

  // working section
  // left, right, left, right
  double buffer[2]; // power, angle
  double wheelPowers[8];
  // call function with arguments a, b, v
  ClosestDir(((hw::frontLeftRotation.GetSelectedSensorPosition() - 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(b,d),RHO(b,d),buffer);
  wheelPowers[0] = buffer[0];
  wheelPowers[1] = ((hw::frontLeftRotation.GetSelectedSensorPosition() - 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::frontRightRotation.GetSelectedSensorPosition() + 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(b,c),RHO(b,c),buffer);
  wheelPowers[2] = buffer[0];
  wheelPowers[3] = ((hw::frontRightRotation.GetSelectedSensorPosition() + 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::backLeftRotation.GetSelectedSensorPosition() + 128 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(a,d),RHO(a,d),buffer);
  wheelPowers[4] = buffer[0];
  wheelPowers[5] = ((hw::backLeftRotation.GetSelectedSensorPosition() + 128 - 512)) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];
  ClosestDir(((hw::backRightRotation.GetSelectedSensorPosition() - 256 - 16 - 512) / 1023) * (2 * constants::TIMSCONSTANT), PHI(a,c),RHO(a,c),buffer);
  wheelPowers[6] = buffer[0];
  wheelPowers[7] = (hw::backRightRotation.GetSelectedSensorPosition() - 256 - 16 - 512) + ((buffer[1] / (2 * 3.1415192)) * 1023);//buffer[1];


  double dogcalibration[4] = {1,1,1,1};//{1,.69420,1,.69420};
  double changeDir = 1;
  // Make wheels stay where they are
  if (Vx * Vx + Vy*Vy + w*w != 0){
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[0] * dogcalibration[0]);
    hw::frontLeftRotation.Set(ControlMode::Position, changeDir * wheelPowers[1] + 128 + 512); // +45

    hw::frontRightDrive.Set(ControlMode::PercentOutput, wheelPowers[2] * dogcalibration[1]);
    hw::frontRightRotation.Set(ControlMode::Position, changeDir * wheelPowers[3] - 128 + 512); // -45

    hw::backLeftDrive.Set(ControlMode::PercentOutput, wheelPowers[4] * dogcalibration[2]);
    hw::backLeftRotation.Set(ControlMode::Position, changeDir * wheelPowers[5] - 128 + 512); // -45

    hw::backRightDrive.Set(ControlMode::PercentOutput, wheelPowers[6] * dogcalibration[3]);
    hw::backRightRotation.Set(ControlMode::Position, changeDir * wheelPowers[7] + 256 + 16 + 512); // +90
  }
  else{
    hw::frontLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::frontRightDrive.Set(ControlMode::PercentOutput, 0);
    hw::backLeftDrive.Set(ControlMode::PercentOutput, 0);
    hw::backRightDrive.Set(ControlMode::PercentOutput,0);
  }
  

  //arm
  int armConstant = .3;//.2;
  if (hw::xBox.GetLeftY()){
    armHeight1.Set(armConstant-.4 * hw::xBox.GetLeftY());
    armHeight2.Set(armConstant-.4 * hw::xBox.GetLeftY());
  }
  else{
    armHeight1.Set(armConstant-.4 * hw::xBox.GetLeftY());
    armHeight2.Set(armConstant-.4 * hw::xBox.GetLeftY());
  }

  double armInput = .7 * hw::xBox.GetRightTriggerAxis() -.7 *  hw::xBox.GetLeftTriggerAxis();
  armExtension.Set(ControlMode::PercentOutput, armInput);
  
  /*
  armExtension.Set(ControlMode::PercentOutput, 0);
  double armInput = .7 * hw::xBox.GetRightTriggerAxis() -.7 *  hw::xBox.GetLeftTriggerAxis();
  if (armInput > 0 ){
    if (armLimitPos <= 4){
      armExtension.Set(ControlMode::PercentOutput, armInput);
    }
  }
  else{
    if (armLimitPos > 0){
      armExtension.Set(ControlMode::PercentOutput, armInput);
    }
  }

  if (hw::xBox.GetLeftBumper()) {
    goingToPos = 2;
  }

  if (hw::xBox.GetRightBumper()) {
    goingToPos = 1;
  }

  if (goingToPos == 1 && armInput <= 4) {
    armExtension.Set(ControlMode::PercentOutput, -.2);
  }
  else if (goingToPos == 2 && armInput > 0) {
    armExtension.Set(ControlMode::PercentOutput, .2);
  }
  else {
    armInput = 0;
  }
  std::cout << armLimitPos << std::endl;
  */


  // update encoder pos 
  /*
  if (armLimit.Get() && !switchHit) {
    if (armExtension.GetMotorOutputPercent() > 0) {
      armLimitPos++;
    }
    else {
      armLimitPos--;
    }
    switchHit = true;
  }
  else {
    if (!armLimit.Get()) {
    switchHit = false;
    }
  }*/

  //flywheel
 // clawMotor0.Set(-hw::xBox.GetRightY());
  //clawMotor1.Set(-hw::xBox.GetRightY());

  
  //pneumatics
  if(hw::xBox.GetXButtonPressed()) claw.Toggle(); //close
  else if(hw::xBox.GetBButtonPressed()) claw.Toggle(); //open
  
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
int main() {
  return frc::StartRobot<Robot>();
}
#endif
