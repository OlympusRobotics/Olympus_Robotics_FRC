// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CameraUsing;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.TurretAiming;

// NOTE: Changes to controller bindings, subsystem wiring, or auto commands here
// must also be reflected in Theseus/README.md (Controller Bindings, Autonomous sections).
public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 1 rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    /* private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); */

    //initalize controllers (joy and whimsey lmao im so funny :3)
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController whimseystick = new CommandXboxController(1);

    //initalize the subsystems
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CameraUsing visioningit = new CameraUsing(drivetrain);
    public final TurretAiming aiming = new TurretAiming(drivetrain);
    public final Intake intake = new Intake();
    private final Field2d field = new Field2d();
    private final SendableChooser<Command> autoChooser;
    public Command m_autonomousCommand;
    //private final Climber climber = new Climber();
    public final McpJoystick mcpJoystick = new McpJoystick(0);


    //Initalize contoller commands
    /** Toggles intake flywheel on/off and deploys arm (LT) */
    private final Command intakeToggle = intake.startEnd(
      () -> { intake.spinflywheel(); },
      () -> { intake.stopspin(); }
    );

    /** Opens and closes intake without controller requiremets*/
    private final Command autoIntake = intake.startEnd(() -> { intake.spinflywheel(); },
      () -> { intake.stopspin(); });

    private final Command lowerintake = intake.startEnd(() -> intake.startIntake(), () -> intake.endIntake());

    /** Closes intake */
    private final Command jerkIntake = intake.startEnd(() -> intake.jerkIntake(), () -> intake.endIntake());
      
    /** Spins the flywheels to shoot a ball */
    private final Command shoot = aiming.startEnd(() -> aiming.shoot(), () -> aiming.unshoot())
      .until(() -> joystick.rightBumper().getAsBoolean() == false); //RT button
    
    private final Command indexshoot = aiming.startEnd(() -> aiming.index(), () -> aiming.unshoot())
      .until(() -> joystick.rightTrigger().getAsBoolean() == false); //RT button

    /** Spins the flywheels to shoot a ball without controller*/

    private final Command autoRev = aiming.startEnd(() -> aiming.autoshoot(), () -> aiming.autoshoot());

    private final Command autoshoot = aiming.startEnd(() -> aiming.autoindex(), () -> aiming.unshoot());

    private final Command startingaim = aiming.startEnd(() -> aiming.startingaim(), () -> aiming.unshoot());

    private final Command endingaim = aiming.startEnd(() -> aiming.startingaim(), () -> aiming.unshoot());

    /** Reverse indexer only (left bumper) */
    private final Command intakingOut = aiming.startEnd(() -> aiming.reverseIndexer(), () -> aiming.stopMotors())
      .until(() -> joystick.leftBumper().getAsBoolean() == false);

    /** Resets the turret */
    public final Command resetsTurret = aiming.startEnd(() -> aiming.resetTurret(), () -> aiming.stopMotors())
      .until(() -> joystick.b().getAsBoolean() == false); //B button


      private final Command limelightAiming = aiming.startEnd(() -> aiming.limelightAim(), () -> aiming.stopMotors())
        .until(() -> joystick.a().getAsBoolean() == false); //Y button

    /*private final Command llAutoAim = new RunCommand(() -> { //stinky stinky limelight stuff

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

    /** 
     * Helper to apply a joystick deadband and rescale the remaining range. 
     * @param value the value to apply deadband to
     * @return the value after the deadband
    */
    private double applyDeadband(double value) {
        final double deadband = 0.05;
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        // Rescale so output starts from 0 after the deadband:
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    public RobotContainer() {
      
        drivetrain.configureAutobuilder();
        aiming.setMcpJoystick(mcpJoystick);
        configureBindings();
        NamedCommands.registerCommand("Rev", autoRev.withTimeout(3));
        NamedCommands.registerCommand("shoot", autoshoot.withTimeout(5));
        NamedCommands.registerCommand("intake", autoIntake.withTimeout(20));
        NamedCommands.registerCommand("startaim", startingaim.withTimeout(6));
        NamedCommands.registerCommand("endaim", endingaim.withTimeout(20));
        NamedCommands.registerCommand("lowerintake", lowerintake.withTimeout(.5));
        NamedCommands.registerCommand("Jerk", jerkIntake.withTimeout(1));
        SmartDashboard.putData("Field", field);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("straight through right", autoChooser);
        SmartDashboard.putData("straight through left", autoChooser);
        SmartDashboard.putData("steal right ", autoChooser);
        SmartDashboard.putData("steal left", autoChooser);
        SmartDashboard.putData("right half x2 score", autoChooser);
        SmartDashboard.putData("left half x2 score", autoChooser);
        SmartDashboard.putData("Bad Grab n' Go", autoChooser);
        SmartDashboard.putData("Good Grab 'n Go", autoChooser);
        SmartDashboard.putData("Back up and Shoot", autoChooser);
        SmartDashboard.putData("New Auto", autoChooser);

        // Override "Zero Turret" to also zero the intake position
        SmartDashboard.putData("Zero Turret", new InstantCommand(() -> {
            aiming.zeroTurret();
            intake.zeroPosition();
        }).ignoringDisable(true));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-this.applyDeadband(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-this.applyDeadband(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(this.applyDeadband(-(RobotBase.isSimulation() ? joystick.getRawAxis(2) : joystick.getRightX())) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        //Binds the commands to the buttons
        joystick.leftBumper().whileTrue(intakingOut);
        new Trigger(() -> Math.abs(joystick.getRightTriggerAxis()) > 0.5).whileTrue(indexshoot);
        joystick.rightBumper().whileTrue(shoot);
        Trigger leftTrigger = new Trigger(() -> Math.abs(joystick.getLeftTriggerAxis()) > 0.5);
        leftTrigger.whileTrue(intakeToggle);
        joystick.b().whileTrue(resetsTurret);
        joystick.a().onTrue(limelightAiming);
        joystick.y().onTrue(aiming.runOnce(() -> aiming.toggleHeadingHold()));
        joystick.start().onTrue(aiming.runOnce(() -> aiming.toggleScoringMode()));
        joystick.back().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
                .alongWith(aiming.runOnce(aiming::disableAllModes))
        );

        // D-pad turret controls: left/right = manual rotate, up/down = manual height
        // MCP simulated joystick is handled directly in TurretAiming.periodic()
        joystick.povLeft().whileTrue(aiming.run(() -> aiming.manualRotate(-1)).finallyDo(() -> aiming.resetManualRamp()));
        joystick.povRight().whileTrue(aiming.run(() -> aiming.manualRotate(1)).finallyDo(() -> aiming.resetManualRamp()));
        joystick.povUp().whileTrue(aiming.run(() -> aiming.manualHeight(1)).finallyDo(() -> aiming.resetManualRamp()));
        joystick.povDown().whileTrue(aiming.run(() -> aiming.manualHeight(-1)).finallyDo(() -> aiming.resetManualRamp()));
        
        //joystick.start().onTrue(aiming.runOnce(() -> {useTurretMotionMagic = !useTurretMotionMagic;}) );

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

        whimseystick.y().whileTrue(drivetrain.sysIdQuasistaticTrans(Direction.kForward));
        whimseystick.a().whileTrue(drivetrain.sysIdQuasistaticTrans(Direction.kReverse));
        whimseystick.b().whileTrue(drivetrain.sysIdDynamicTrans(Direction.kForward));
        whimseystick.x().whileTrue(drivetrain.sysIdDynamicTrans(Direction.kReverse));
        whimseystick.povDown().whileTrue(drivetrain.sysIdQuasistaticRot(Direction.kForward));
        whimseystick.povUp().whileTrue(drivetrain.sysIdQuasistaticRot(Direction.kReverse));
        whimseystick.povRight().whileTrue(drivetrain.sysIdDynamicRot(Direction.kForward));
        whimseystick.povLeft().whileTrue(drivetrain.sysIdDynamicRot(Direction.kReverse));
        whimseystick.leftTrigger().whileTrue(drivetrain.sysIdDynamicSteer(Direction.kForward));
        whimseystick.leftBumper().whileTrue(drivetrain.sysIdDynamicSteer(Direction.kReverse));
        whimseystick.rightTrigger().whileTrue(drivetrain.sysIdQuasistaticSteer(Direction.kForward));
        whimseystick.rightBumper().whileTrue(drivetrain.sysIdQuasistaticSteer(Direction.kReverse));


        // Reset the field-centric heading on left bumper press.

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
          Command selectedAuto = autoChooser.getSelected();
          if (selectedAuto != null) {
              return selectedAuto;
          }
        return new PathPlannerAuto("New New Auto");
    } 
    public void periodic() {
      SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    }
}

