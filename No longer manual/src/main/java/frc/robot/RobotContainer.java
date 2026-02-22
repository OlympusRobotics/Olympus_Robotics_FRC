// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CameraUsing;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.TurretAiming;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    /* private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); */

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController whimseystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CameraUsing visioningit = new CameraUsing(drivetrain);
    public final TurretAiming aiming = new TurretAiming(drivetrain);

    private final Intake intake = new Intake();
    //private final Climber climber = new Climber();

    private final Command Intake = intake.startEnd(() -> intake.startIntake(), () -> intake.endIntake()) //Opens and closes the intake respectivly
      .until(() -> joystick.getLeftTriggerAxis() <= .5); //pressing the LT button

      //TEMPORARILY configured to the shoot instead of climb
    private final Command shoot = aiming.startEnd(() -> aiming.shoot(), () -> aiming.unshoot()) //extends and retracts the climber
      .until(() -> joystick.rightTrigger().getAsBoolean() == false); //RT button

    private final Command intakingOut = Commands.parallel(intake.startEnd(() -> intake.outakeIntake(), () -> intake.endIntake()), aiming.startEnd(() -> aiming.reverseIndexer(), () -> aiming.stopMotors()))
      .until(() -> joystick.leftBumper().getAsBoolean() == false);

    public final Command locksTurret = aiming.startEnd(() -> aiming.lockTurret(), () -> aiming.stopMotors()) //locks the aiming 🤯
      .until(() -> joystick.x().getAsBoolean() == false); //X button

      public final Command resetsTurret = aiming.startEnd(() -> aiming.resetTurret(), () -> aiming.stopMotors()) //lowers height down 🤯
      .until(() -> joystick.b().getAsBoolean() == false); //X button

    private final Command unlocksTurret = aiming.startEnd(() -> aiming.targetAim(), () -> aiming.targetAim()) //when pressed will activate aimbot :3
      .until(() -> joystick.a().getAsBoolean() == false); //Start button

    /*private final Command llAutoAim = new RunCommand(() -> { //limelight aimbot :3

        double forwardVal = applyDeadband(limes.aimAndRange()[0]);
        double strafeVal = applyDeadband((-joystick.getLeftX())); // Negate to match joystick direction
        double rotationVal = applyDeadband(limes.aimAndRange()[1]);

        if (forwardVal != 0 || strafeVal != 0 || rotationVal != 0)
          m_drivetrain.drive(forwardVal*4.1, strafeVal*4.1, rotationVal*4.1, false);
        else m_drivetrain.stopmotors();
      }, drivetrain)
    .until(() -> joystick.y().getAsBoolean() == false); //on the y button*/

    /* private final Command resetYaw = drivetrain.startEnd(() -> drivetrain.seedFieldCentric(), () -> drivetrain.seedFieldCentric()) //resets the yaw
      .until(() -> joystick.back().getAsBoolean() == false); //start button */

    // Helper to apply a joystick deadband and rescale the remaining range.
    private double applyDeadband(double value) {
        final double deadband = 0.05;
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        // Rescale so output starts from 0 after the deadband:
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(this.applyDeadband(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(this.applyDeadband(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(this.applyDeadband(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        joystick.leftBumper().whileTrue(intakingOut);
        new Trigger(() -> Math.abs(joystick.getLeftTriggerAxis()) > 0.5).whileTrue(Intake);
        new Trigger(() -> Math.abs(joystick.getRightTriggerAxis()) > 0.5).whileTrue(shoot);
        joystick.x().whileTrue(locksTurret);
        joystick.b().whileTrue(resetsTurret);
        joystick.a().whileTrue(unlocksTurret);
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        joystick.povLeft().whileTrue(intakingOut);
        /* joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
 */
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); */

        whimseystick.leftBumper().onTrue(drivetrain.runOnce(() -> SignalLogger.start()));
        whimseystick.rightBumper().onTrue(drivetrain.runOnce(() -> SignalLogger.stop()));

        whimseystick.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        whimseystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        whimseystick.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        whimseystick.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        

        // Reset the field-centric heading on left bumper press.

        drivetrain.registerTelemetry(logger::telemeterize);


        NamedCommands.registerCommand("shoot", shoot);
        NamedCommands.registerCommand("intake", Intake);
    }

    public Command getAutonomousCommand() {
        
        // Load the path you want to follow using its name in the GUI
        return new PathPlannerAuto("New Auto");
    } 
}
