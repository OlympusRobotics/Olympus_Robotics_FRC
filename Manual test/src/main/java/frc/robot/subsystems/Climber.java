// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  final TalonFX m_Climber; // One climber motor for now
  private final PIDController m_ClimberController = new PIDController(0.7, 0, 0);

  // Neo PID constants
  final Double kP = 0.7; final Double kI = 0.0; final Double kD = 0.0; final Double kIz = 0.0; final Double kFF = 0.0; final Double kMaxOutput = 1.0; final Double kMinOutput = -1.0;

  // ClimberSetpoint
  final Double resting = 0.0;
  final Double fullyExtended = 64.0;

  /** Wow its the climber */
  public Climber() {

    m_Climber = new TalonFX(20);

    // set PID constants
    m_ClimberController.setPID(kP, kI, kD);
    m_ClimberController.setIZone(kIz);
    // m_ClimberController.setFF(kFF);
    m_ClimberController.setIntegratorRange(kMinOutput, kMaxOutput);

  }
/*
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⣤⣤⣤⣶⣶⣶⣶⠶⢠⣤⣤⣤⣤⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣴⣾⠛⠛⠋⠉⠉⠉⠀⠀⠀⠀⠀⠈⠉⠉⠉⠉⠉⠛⣿⣶⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣰⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣤⣀⠀⠀⠀⠀⠙⠻⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⠀⠀⠀⠀⠀⠀⢀⣤⡀⠀⠀⠀⠀⣀⣀⣴⣿⠛⠉⠻⢿⣦⡀⠀⠀⠀⠘⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⣶⡶⠿⢶⢶⣶⣶⣾⠟⠋⠀⠀⠀⠘⠛⠛⠋⠀⠀⠀⠀⠈⠿⠃⠀⠀⠀⠀⠹⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⣿⠇⠀⢀⠀⠀⢀⣀⡀⡀⠀⠀⠤⠀⠀⠀⠀⠀⢨⠀⠀⠀⠀⠀⠀⠀⠀⠤⠀⠀⠀⠀⠀⠸⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢸⡿⠀⠀⣿⠿⠿⠟⠛⣿⣿⣿⣿⡿⠂⠀⠀⠀⠀⠺⠿⠿⣿⣿⣿⣿⡿⠿⢿⣾⠀⠀⠀⠀⠀⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢸⡇⠠⠀⠀⠀⠀⠀⠀⠙⠿⠿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⠿⠿⠁⠀⠀⠀⠀⠀⠀⠀⠸⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣼⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣾⣿⣿⡿⢿⣦⠀⠀⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠸⣷⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⠇⠀⠁⠀⢀⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢻⣷⡀⠀⠀⠀⢀⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣠⣴⡾⠟⠁⠀⠀⠀⢀⣾⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣷⡀⠀⠀⠈⢿⣿⣶⣶⣶⣶⣆⣀⣀⣰⣶⣶⣶⣾⣿⢿⠿⠁⠀⠀⠀⠀⠀⢀⣾⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⣷⣄⠀⠀⠀⠀⠈⠈⠉⠉⢩⣭⣉⣉⣉⣉⣠⣤⣶⡾⠃⠀⠀⠀⠀⠀⣲⡿⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣷⣄⣀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠋⠉⠉⠁⠀⠀⠀⠀⠀⣀⣴⣾⣟⣁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣈⣿⠿⠿⢶⣶⣤⣤⣤⣄⣀⣀⣠⣤⣤⣤⣴⣶⡶⠿⠿⠛⠋⠈⠛⠻⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⡟⠙⠀⠀⠀⠀⠈⠉⠉⠉⠋⠙⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣷⡤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣿⣆⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⡟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣦⣀⠀⢰⣄⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢿⣦⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣰⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⠛⠉⢻⣶⣿⠋⠛⢿⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣧⡀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⢀⣼⡿⠉⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣤⡶⠿⠛⠛⠛⠻⣦⡈⣿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣷⣀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢠⣾⠟⠀⠀⠀⠀⢰⣿⡆⠀⠀⣠⣶⠾⡟⢻⣩⣼⣧⣦⣤⣤⣀⣀⣀⣿⣿⣿⣿⣦⡄⠀⠀⠀⣀⣦⠀⠀⠀⠀⠀⢻⣷⡀⠀⠀⠀⠀
⠀⠀⠀⢰⣿⠃⠀⠀⠀⠀⠀⠀⣿⡇⠀⣴⡿⠁⢈⣴⠟⠉⠀⢀⣀⣤⣭⣿⣭⣍⡙⠀⠀⠉⠻⣿⣴⡀⠀⣿⣿⠀⠀⠀⠀⠀⠀⠻⣷⡀⠀⠀⠀
⠀⠀⢠⣿⠇⠀⠀⠀⠀⠀⠀⠠⣿⣅⣼⠟⠀⠀⠺⣿⣤⣴⠾⠟⠋⠉⠀⠀⠉⠙⠻⢷⣦⡀⠀⠸⣿⣧⠀⢸⣿⠀⠀⠀⠀⠀⠀⠐⢻⣿⡀⠀⠀
⠀⠀⣼⡿⢀⠀⠀⠀⠀⠀⠀⢰⣿⣿⠟⠀⠀⠀⠀⠻⠃⠀⠀⣀⣀⣶⣤⣄⣀⠀⠀⣀⣿⡇⠀⠀⢸⣿⡆⢸⣿⠀⠀⠀⠀⠀⠀⠀⠀⢿⣧⠀⠀
⠀⣼⡿⠁⠀⠀⠀⠀⠀⠀⢀⣼⡿⠁⠀⠀⠀⠀⠀⣠⣶⠾⠿⠛⠛⠉⠉⠙⠛⠿⠟⠛⠋⠀⠀⠀⠀⠙⣿⣼⣿⡇⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀
⢠⣿⠁⠀⠀⢠⣷⣶⣶⡾⠿⠛⠁⠀⠀⠀⠀⠀⠀⣿⣧⣄⡀⢀⣠⣤⣤⣤⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠈⢿⣿⣧⣶⣄⣦⡀⠀⠀⠀⠘⣿⡏⠀
⣾⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠛⠋⠉⠀⠈⠉⠙⢿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠙⠉⠀⠀⠀⠀⠀⢻⣷⠀
⣿⡅⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⡿⠿⠿⠿⠷⣶⣦⣤⣀⣀⣹⣯⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠄
⢿⣿⣶⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣇⣀⡀⠀⢀⣀⣀⣀⣩⣍⣛⠋⡁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⠀
*/

  /** Extends the motor */
  public void extend() {
    // check if motors are too hot
    if (tempProtect(m_Climber) > 0) return;

    m_Climber.setPosition(fullyExtended);
    // self.leftClimber.set(.1)
    // self.rightClimber.set(.1)
  }

  /** Retracts the motor */
  public void retract() {
    // check if motors are too hot
    if (tempProtect(m_Climber) > 0) return;

    m_Climber.setPosition(resting);
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
}