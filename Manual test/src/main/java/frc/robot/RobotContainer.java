// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LLVision;
import frc.robot.subsystems.TurretAiming;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LLVision limes = new LLVision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Intake intake = new Intake();
  private final TurretAiming turret = new TurretAiming();
  private final Climber climber = new Climber();
  private final Command Intake = intake.startEnd(() -> intake.startIntake(), () -> intake.endIntake())
    .until(() -> m_driverController.getLeftTriggerAxis() <= .5);
  private final Command climberer = climber.startEnd(() -> climber.extend(), () -> climber.retract())
    .until(() -> m_driverController.leftBumper().getAsBoolean() == false);
  private final Command intakeOut = intake.startEnd(() -> intake.outakeIntake(), () -> intake.endIntake())
    .until(() -> m_driverController.rightBumper().getAsBoolean() == false);
  private final Command locksTurret = turret.startEnd(() -> turret.lockTurret(), () -> turret.stopMotors())
    .until(() -> m_driverController.x().getAsBoolean() == false);
  private final Command unlocksTurret = turret.startEnd(() -> turret.targetAim(), () -> turret.targetAim())
    .until(() -> m_driverController.b().getAsBoolean() == false);
  private final Command llAutoAim = new RunCommand(() -> {

        double forwardVal = applyDeadband(limes.aimAndRange()[0]); // Negate to match joystick direction
        double strafeVal = applyDeadband((-m_driverController.getLeftX()));
        double rotationVal = applyDeadband(limes.aimAndRange()[1]);


        if (forwardVal != 0 || strafeVal != 0 || rotationVal != 0) {
          Drivetrain.chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            forwardVal*4.1, strafeVal*4.1, rotationVal*4.1,
            m_drivetrain.getRobotRotation()
          );
          m_drivetrain.drive(forwardVal*4.1, strafeVal*4.1, rotationVal*4.1, true);
        } else {
          m_drivetrain.stopmotors();
        }
      }, m_drivetrain)
    .until(() -> m_driverController.y().getAsBoolean() == false);
  private final Command resetYaw = m_drivetrain.startEnd(() -> m_drivetrain.resetYaw(), () -> m_drivetrain.resetYaw())
    .until(() -> m_driverController.start().getAsBoolean() == false);
  
  private Command aimAndDrive() {
    return Commands.run(()->{

      double[] speeds = limes.aimAndRange();
      double xSpeed = speeds[0] * 4;
      //double ySpeed = speeds[1] * 4;
      double rot = speeds[1] * 4;
  
      //double forwardVal = applyDeadband(-m_driverController.getLeftY()); // Negate to match joystick direction
      double strafeVal = applyDeadband(m_driverController.getLeftX());
      //double rotationVal = applyDeadband(m_driverController.getRightX());

      if (strafeVal != 0) {
        ChassisSpeeds speedes = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, strafeVal, rot,
          m_drivetrain.getRobotRotation()
        );
        m_drivetrain.drive(xSpeed, strafeVal, rot, false);
      } else {
        m_drivetrain.stopmotors();
      }
    });
  }

  public void configureBindings() {
    m_driverController.leftBumper().whileTrue(climberer);
    new Trigger(() -> Math.abs(m_driverController.getLeftTriggerAxis()) > 0.5).whileTrue(Intake);
    m_driverController.rightBumper().whileTrue(intakeOut);
    m_driverController.x().whileTrue(locksTurret);
    m_driverController.b().whileTrue(unlocksTurret);
    m_driverController.y().whileTrue(llAutoAim);
    m_driverController.start().whileTrue(resetYaw);
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default command to drive based on joystick input

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> {

        double forwardVal = applyDeadband(-m_driverController.getLeftY()); // Negate to match joystick direction
        double strafeVal = applyDeadband(-m_driverController.getLeftX());
        double rotationVal = applyDeadband(m_driverController.getRightX());


        if (forwardVal != 0 || strafeVal != 0 || rotationVal != 0) {
          Drivetrain.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardVal*4.1, strafeVal*4.1, rotationVal*4.1,
            m_drivetrain.getRobotRotation()
          );
          m_drivetrain.drive(forwardVal*4.1, strafeVal*4.1, rotationVal*4.1, true);
        } else {
          m_drivetrain.stopmotors();
        }
      }, m_drivetrain) // require the drivetrain
    );
  };
    

    
  

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