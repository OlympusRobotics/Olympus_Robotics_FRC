// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//this is the file where all of the actual commands are held

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
//import com.fasterxml.jackson.databind.util.Named;
//import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import edu.wpi.first.math.filter.SlewRateLimiter;
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
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.LLVision;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooting;

// NOTE: Changes to controller bindings, subsystem wiring, or auto commands here
// must also be reflected in Theseus/README.md (Controller Bindings, Autonomous sections).
public class RobotContainer {
    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 1 rotation per second max angular velocity
    /* private final SlewRateLimiter xLimiter = new SlewRateLimiter(8);  
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(8); 
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(RotationsPerSecond.of(4).in(RadiansPerSecond));  */
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    /* private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); */

    //initalize controllers (joy and whimsey lmao im so funny :3)
    public final CommandXboxController joystick = new CommandXboxController(0);
    //private final CommandXboxController whimseystick = new CommandXboxController(1);

    //initalize the subsystems
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    //public final CameraUsing visioningit = new CameraUsing(drivetrain);
    public final Shooting turret = new Shooting(drivetrain);
    public final Intake intake = new Intake();
    private final Field2d field = new Field2d();
    //private final LLVision limelight = new LLVision(drivetrain, turret);
    private final SendableChooser<Command> autoChooser;
    public Command m_autonomousCommand;
    //private final Climber climber = new Climber();
    public final McpJoystick mcpJoystick = new McpJoystick(0);

    public Boolean maybeHenryDidItWrong = false;
    public double speedMulti = 1.0;
    public double rotMulti = .8;


    //Initalize contoller commands
    /** Toggles intake flywheel on/off and deploys arm (LT) */
    private final Command intakeToggle = intake.startEnd( //this runs the first command upon toggle on and runs the second command upon toggle off
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
    /* private final Command shoot = turret.startEnd(() -> turret.shoot(), () -> turret.unshoot())
      .until(() -> joystick.rightBumper().getAsBoolean() == false); //RT button */
    
    private final Command indexshoot = turret.startEnd(() -> turret.index(), () -> turret.unshoot()) //this runs the first command untill the boolean is true where it will finally run the second command
      .until(() -> joystick.rightTrigger().getAsBoolean() == false); //RT button
    
    private final Command reverseIndexer = turret.startEnd(() -> turret.reverseIndexer(), () -> turret.stopMotors())
    .until(() -> joystick.rightBumper().getAsBoolean() == false);

    /** Spins the flywheels to shoot a ball without controller*/

    private final Command autoRev = turret.startEnd(() -> turret.autoshoot(), () -> turret.autoshoot());

    private final Command autoshoot = turret.startEnd(() -> turret.autoindex(), () -> turret.unshoot());

    /** Reverse indexer only (left bumper) */
    /* private final Command intakingOut = turret.startEnd(() -> turret.reverseIndexer(), () -> turret.stopMotors())
      .until(() -> joystick.leftBumper().getAsBoolean() == false); */

    private final Command intakeOut = intake.startEnd(() -> intake.outakeIntake(), () -> intake.stopspin())
      .until(() -> joystick.leftBumper().getAsBoolean() ==false);

    /** Resets the turret */
    public final Command resetsTurret = turret.startEnd(() -> turret.resetTurret(), () -> turret.stopMotors())
      .until(() -> joystick.b().getAsBoolean() == false); //B button

    private final Command locksTurret = turret.startEnd(() -> turret.lockTurret(), () -> turret.stopMotors()) //locks the turret 🤯
    .until(() -> joystick.x().getAsBoolean() == false); //X button
    //private final Command move = turret.startEnd(() -> turret.manualRotate(), () -> turret.stopMotors());

      //private final Command autoaim = turret.startEnd(() -> turret.limelightAim(), () -> turret.stopMotors());


      private final Command limelightAiming = turret.startEnd(() -> {turret.limelightAim(); speedMulti = 1; rotMulti = 1;}, () -> {turret.unshoot(); speedMulti = 1.0; rotMulti = 1.0;});


      private final Command autolimelight = turret.startEnd(() -> {turret.autolimelightAim();}, () -> {turret.unshoot();});
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
    private double applyDeadband(double value) { //just makes it so if the controller is giving a value less than .08, say stick drift, it will ignore that value
        final double deadband = 0.08;
        if (Math.abs(value) <= deadband) {
            return 0.0;
        }
        // Rescale so output starts from 0 after the deadband:
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }

    public RobotContainer() {
      
      //DriverStation.JoystickConnectionWarningSilenced(true);
        //DriverStation.silenceJoystickConnectionWarning(true);
        drivetrain.configureAutobuilder(); //configures auto
        turret.setMcpJoystick(mcpJoystick); //creates dougs simulated controller
        configureBindings(); //creates the commands

        //Initalize Operator Stuff
        NamedCommands.registerCommand("Rev", autoRev.withTimeout(3)); //all of these are autonomous commands, runs until the timeout 
        NamedCommands.registerCommand("shoot", autoshoot.withTimeout(5));
        NamedCommands.registerCommand("intake", autoIntake.withTimeout(7));
        NamedCommands.registerCommand("lowerintake", lowerintake.withTimeout(1));
        NamedCommands.registerCommand("Jerk", jerkIntake.withTimeout(.35));
        NamedCommands.registerCommand("limelight aim", autolimelight.withTimeout(5));
        SmartDashboard.putData("Field", field);

        //Select Auto
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Back up and Shoot", autoChooser); //this is a list of every choosable auto, the actual code for the autos are built into PathPlanner
        SmartDashboard.putData("Disrupt", autoChooser);
        SmartDashboard.putData("Full disrupt", autoChooser);
        SmartDashboard.putData("Full Disrupt Right", autoChooser);
        SmartDashboard.putData("Disrupt Right", autoChooser);
        SmartDashboard.putData("New Auto", autoChooser);
        SmartDashboard.putData("Grab 'n Go", autoChooser);
        SmartDashboard.putData("Shoot then Depot", autoChooser);
        SmartDashboard.putData("Block Left", autoChooser);
        SmartDashboard.putData("Block Right", autoChooser);
        // Override "Zero Turret" to also zero the intake position
        SmartDashboard.putData("Zero Turret", new InstantCommand(() -> {
            turret.zeroTurret/*  */();
            intake.zeroPosition();
        }).ignoringDisable(true));
    }

    private void configureBindings() { //controller keybinds
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand( //this is how the robot ultimately drives
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-this.applyDeadband(-joystick.getLeftY()) * MaxSpeed * speedMulti) // Drive forward with negative Y (forward)
                    .withVelocityY(-this.applyDeadband(-joystick.getLeftX()) * MaxSpeed * speedMulti) // Drive left with negative X (left)
                    .withRotationalRate(this.applyDeadband(-(RobotBase.isSimulation() ? joystick.getRawAxis(2) : joystick.getRightX())) * MaxAngularRate * rotMulti) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        //Binds the commands to the controlelr buttons

        //joystick.leftBumper().whileTrue(intakingOut);
        joystick.leftBumper().whileTrue(intakeOut); //runs as long as the bumper is held
        new Trigger(() -> Math.abs(joystick.getRightTriggerAxis()) > 0.5).whileTrue(indexshoot);
        joystick.rightBumper().whileTrue(reverseIndexer);
        Trigger leftTrigger = new Trigger(() -> Math.abs(joystick.getLeftTriggerAxis()) > 0.5);
        leftTrigger.whileTrue(intakeToggle);
        joystick.b().whileTrue(resetsTurret);
        joystick.a().toggleOnTrue(limelightAiming); //changes to the start on press and then the end on a second press
        joystick.x().whileTrue(locksTurret);
        joystick.y().onTrue(turret.runOnce(() -> turret.toggleHeadingHold())); //runs start on rising edge then switches almost immediately to the end command
        joystick.start().onTrue(turret.runOnce(() -> turret.toggleScoringMode()));
        joystick.back().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
                .alongWith(turret.runOnce(turret::disableAllModes))
        );

        // D-pad turret controls: left/right = manual rotate, up/down = manual height
        // MCP simulated joystick is handled directly in TurretAiming.periodic()
       
        joystick.povLeft().whileTrue(turret.run(() -> turret.manualRotate(-1)).finallyDo(() -> turret.resetManualRamp())); // after running the first command finally run the second command
        joystick.povRight().whileTrue(turret.run(() -> turret.manualRotate(1)).finallyDo(() -> turret.resetManualRamp()));
        joystick.povUp().whileTrue(turret.run(() -> turret.manualHeight(1)).finallyDo(() -> turret.resetManualRamp()));
        joystick.povDown().whileTrue(turret.run(() -> turret.manualHeight(-1)).finallyDo(() -> turret.resetManualRamp()));
        
        //joystick.start().onTrue(turret.runOnce(() -> {useTurretMotionMagic = !useTurretMotionMagic;}) );

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

        /* joystick.leftBumper().onTrue(drivetrain.runOnce(() -> SignalLogger.start()));
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> SignalLogger.stop())); */

        /* joystick.y().whileTrue(TurretAiming.flySysid.quasistatic(Direction.kForward));
        joystick.a().whileTrue(TurretAiming.flySysid.quasistatic(Direction.kReverse));
        joystick.b().whileTrue(TurretAiming.flySysid.dynamic(Direction.kForward));
        joystick.x().whileTrue(TurretAiming.flySysid.dynamic(Direction.kReverse)); */
        
        //joystick.y().whileTrue(drivetrain.sysIdQuasistaticTrans(Direction.kForward));
        //joystick.a().whileTrue(drivetrain.sysIdQuasistaticTrans(Direction.kReverse));
        //joystick.b().whileTrue(drivetrain.sysIdDynamicTrans(Direction.kForward));
        //joystick.x().whileTrue(drivetrain.sysIdDynamicTrans(Direction.kReverse));
        /* joystick.povDown().whileTrue(drivetrain.sysIdQuasistaticRot(Direction.kForward));
        joystick.povUp().whileTrue(drivetrain.sysIdQuasistaticRot(Direction.kReverse));
        joystick.povRight().whileTrue(drivetrain.sysIdDynamicRot(Direction.kForward));
        joystick.povLeft().whileTrue(drivetrain.sysIdDynamicRot(Direction.kReverse));
        joystick.leftTrigger().whileTrue(drivetrain.sysIdDynamicSteer(Direction.kForward));
        joystick.leftBumper().whileTrue(drivetrain.sysIdDynamicSteer(Direction.kReverse));
        joystick.rightTrigger().whileTrue(drivetrain.sysIdQuasistaticSteer(Direction.kForward));
        joystick.rightBumper().whileTrue(drivetrain.sysIdQuasistaticSteer(Direction.kReverse)); */


        // Reset the field-centric heading on left bumper press.

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command getAutonomousCommand() { //sets the selected auto to whatever is selected on Pathplanner/Elastic
        // Load the path you want to follow using its name in the GUI
          Command selectedAuto = autoChooser.getSelected();
          if (selectedAuto != null) {
              return selectedAuto;
          }
        return new PathPlannerAuto("New Auto");
    } 
    public void periodic() { //you can see battery voltage on driver station no clue why we have it here as well
      SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    }
}

