// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Drivetrain m_drivetrain;
  private final CommandXboxController m_driverController;
  
  public RobotContainer() {
    // Set the default command to drive based on joystick input
    m_drivetrain = new Drivetrain();
    m_driverController = new CommandXboxController(Constants.DriverController);

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> {
        double forwardVal = applyDeadband(m_driverController.getLeftY()); // Negate to match joystick direction
        double rotationVal = applyDeadband(m_driverController.getRightX());

        if (forwardVal != 0 || rotationVal != 0) {
          m_drivetrain.drive(forwardVal, rotationVal);
        } else {
          m_drivetrain.stopDrive();
        }
      }, m_drivetrain) // require the drivetrain
    );
    
  }

  private double applyDeadband(double value) {
    if (Math.abs(value) < 0.08) {
      return 0;
    }
    return value;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
