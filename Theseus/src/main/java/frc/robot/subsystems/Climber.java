// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Our robot never actually had a climber, this was just translated code from the 2024 robot "Perry"

package frc.robot.subsystems;
// NOTE: Changes to motor config or setpoints must be reflected in Theseus/README.md (Climber section).
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  final TalonFX m_Climber; // One climber motor for now
  private final PIDController m_ClimberController = new PIDController(0.7, 0, 0);

  // Neo PID constants
  final Double kP = 0.7; final Double kI = 0.0; final Double kD = 0.0; final Double kIz = 0.0; 
  final Double kFF = 0.0; final Double kMaxOutput = 1.0; final Double kMinOutput = -1.0;

  // ClimberSetpoint
  final Double resting = 0.0;
  final Double fullyExtended = 64.0;

  /** Wow its the climber */
  public Climber() {

    m_Climber = new TalonFX(RobotConstants.kClimberID);

    // set PID constants
    m_ClimberController.setPID(kP, kI, kD);
    m_ClimberController.setIZone(kIz); //this does nothing since kIz is 0 but basically tones down how much control I has because it can be a bit controlling
    // m_ClimberController.setFF(kFF);
    m_ClimberController.setIntegratorRange(kMinOutput, kMaxOutput); //more tuning down I control but I is still 0, Jason Wang had no idea what it did and we just translated it since we never actually used it

  }

  /** Extends the motor */
  public void extend() {
    // check if motors are too hot
    if (tempProtect(m_Climber) > 0) return; //during Perry motor overheating was a concern, this stops the motor from moving if it is too hot, tempProtect returns a value of 1 if overheating

    m_Climber.set(m_ClimberController.calculate(m_Climber.get(), fullyExtended)); //follow the PID controlto move the climber from its current position to the wanted position in rotations usually
  }

  /** Retracts the motor */
  public void retract() {
    // check if motors are too hot
    if (tempProtect(m_Climber) > 0) return;
    m_Climber.set(m_ClimberController.calculate(m_Climber.get(), resting));
   
  }

  /** Perry util function, gets if the motor is over 100° */
  public int tempProtect(TalonFX motorController) {
    // Check if motor temp is too high before doing stuff, nonzero eror code means
    // motor too hot"""
    double temp = motorController.getDeviceTemp().getValueAsDouble();
    if (temp > 100) {
      motorController.set(0);
      return 1;
    }
    return 0;
  }

  @Override
  public void periodic() { //runs every 20 ms but would just debug
    //Logger.recordOutput("Climber/Position", m_Climber.getPosition().getValueAsDouble());
    //Logger.recordOutput("Climber/Temperature", m_Climber.getDeviceTemp().getValueAsDouble());
  }
}