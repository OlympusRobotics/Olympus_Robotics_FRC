// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default command to drive based on joystick input
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> {
        double forwardVal = applyDeadband(-m_driverController.getLeftY()); // Negate to match joystick direction
        double strafeVal = applyDeadband(-m_driverController.getLeftX());
        double rotationVal = applyDeadband(m_driverController.getRightX());

        if (forwardVal != 0 || strafeVal != 0 || rotationVal != 0) {
          ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardVal*4.1, strafeVal*4.1, rotationVal*4.1,
            m_drivetrain.getRobotRotation()
          );
          m_drivetrain.drive(speeds);
        } else {
          m_drivetrain.stopDrive();
        }
      }, m_drivetrain) // require the drivetrain
    );
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private double applyDeadband(double value) {
    if (Math.abs(value) < 0.1) {
      return 0;
    }
    return value;
  }

  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

}
