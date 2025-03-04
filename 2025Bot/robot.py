import commands2
import ntcore
import pathplannerlib.config
import wpilib
import wpilib.simulation
import wpilib.drive
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import pathplannerlib
import util.elasticlib as elasticlib
from Subsystems.drivetrain import Drivetrain
from Subsystems.Limelight import limelight
from Subsystems.LED import led
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from Subsystems.elevator import Elevator
from Subsystems.AlgaeArm import algaeArm
from Subsystems.Climber import climber

drivetrain = Drivetrain()

#Autobuilder configures the settings for accurate auto following. Called outside of class to avoid multiple instances of the drivetrain being created.
AutoBuilder.configure(
    drivetrain.getPose,
    drivetrain.resetPose,
    drivetrain.getChassisSpeeds,
    lambda speeds, feedforwards: drivetrain.drive(speeds),
    PPHolonomicDriveController(
        PIDConstants(5.3, 0.0, 0.0),
        PIDConstants(5.6, 0.0, 0.0),
    ),
    RobotConfig.fromGUISettings(),
    drivetrain.shouldFlipPath,
    drivetrain
    ) 


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.elevator = Elevator()
        self.drivetrain = drivetrain
        self.limelight = limelight()
        self.algaeArm = algaeArm()
        self.climber = climber()
        self.led = led()
        
        self.DS = wpilib.DriverStation
        self.chooser = wpilib.SendableChooser()

        self.chooser.addOption("Test1", None)
        self.chooser.addOption("Test2", None)
        self.chooser.addOption("Test3", None)
        self.chooser.addOption("Test4", None)

        self.chooser.setDefaultOption("Test1", None)

        wpilib.SmartDashboard.putData("Auto Options", self.chooser)

        #Auto selection
        self.square = "Test"
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        #SwerveModule states are configured to publish to networkTables for logging and tracking. 
        swerveStates = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.swervePublish = swerveStates.publish()
        
        #Initializes a field for odometry tracking.
        self.field = wpilib.Field2d()

        
        #Commands
        self.algaeEjectReturnHome = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.algaeArm.algaeEject, self).andThen(commands2.WaitCommand(1)),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor, self),
            commands2.InstantCommand(self.algaeArm.setHomePosition, self)
        )

        self.algaeIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.algaeArm.setIntakePosition, self),
            commands2.InstantCommand(self.algaeArm.intake, self),
            commands2.WaitCommand(1),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor, self)
        )
        
        self.intakeCoralTransferL1 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL1, self)
        )
        
        self.intakeCoralTransferL2 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL2, self)
        )
        
        self.intakeCoralTransferL3 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL3, self)
        )
        
        self.intakeCoralTransferL4 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL4, self)
        )

        self.coralEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self)
        )

        self.climbFinal = commands2.SequentialCommandGroup(
            commands2.ParallelCommandGroup(
            commands2.InstantCommand(self.climber.setHomePosition, self),
            commands2.InstantCommand(self.elevator.setHome, self)
            ),
            commands2.InstantCommand(self.climber.setClimbPosition, self),
        )

        #sending commands above to Pathplanner
        NamedCommands.registerCommand("AlgaeEjectReturnHome", self.algaeEjectReturnHome)
        NamedCommands.registerCommand("AlgaeIntake", self.algaeIntake)
        NamedCommands.registerCommand("IntakeCoralTransferL1", self.intakeCoralTransferL1)
        NamedCommands.registerCommand("IntakeCoralTransferL2", self.intakeCoralTransferL2)
        NamedCommands.registerCommand("IntakeCoralTransferL3", self.intakeCoralTransferL3)
        NamedCommands.registerCommand("IntakeCoralTransferL4", self.intakeCoralTransferL4)
        NamedCommands.registerCommand("CoralEject", self.coralEject)
        NamedCommands.registerCommand("ClimbFinal", self.climbFinal)


        #Creates and starts a timer object.
        self.timer = wpilib.Timer()
        self.timer.start()

    def robotPeriodic(self): 
        """ 
        Method that runs every 20ms regardless of what mode the robot is in.
        """
        
        #Always updates the current position and state of the robot and swerve modules respectively.
        self.drivetrain.updateOdometry()  
        self.swervePublish.set([self.drivetrain.flSM.getState(),self.drivetrain.frSM.getState(),self.drivetrain.blSM.getState(),self.drivetrain.brSM.getState()]) 
        
        #Creates an array of the motor temperatures.
        self.driveTrainDeviceTemps = [
            self.drivetrain.flSM.driveMotor.get_device_temp().value_as_double, self.drivetrain.flSM.rotationMotor.getMotorTemperature(),
            self.drivetrain.frSM.driveMotor.get_device_temp().value_as_double, self.drivetrain.frSM.rotationMotor.getMotorTemperature(),
            self.drivetrain.blSM.driveMotor.get_device_temp().value_as_double, self.drivetrain.blSM.rotationMotor.getMotorTemperature(),
            self.drivetrain.brSM.driveMotor.get_device_temp().value_as_double, self.drivetrain.brSM.rotationMotor.getMotorTemperature(),
            self.drivetrain.gyro.get_temperature().value_as_double
            ]   
        wpilib.SmartDashboard.putBoolean("Target", self.limelight.targetCheck())
        
        #Adds the robot pose to the field that was constructed in robotInit.
        wpilib.SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.drivetrain.odometry.getPose())

        #Publishes the hardware telemetry values to SmartDashboard.
        self.getBatteryVoltage()
        self.getTemps()
        self.getMatchTime()
       

        return super().robotPeriodic()
        
    def applyDeadband(self, value, deadband=0.08):
        """ 
        Applys a deadband to stop controller drifting.
        """
        return value if abs(value) > deadband else 0

 
    def getAutoCommand(self):
        """ 
        Selects the auto to be run.
        """
        self.autoSelected = self.square
        auto = PathPlannerAuto(self.autoSelected)

        return auto
    
    def autonomousInit(self):
        """ 
        Runs the auto selection.
        """
        self.autoCommand = PathPlannerAuto("Test")

        self.autoCommand.schedule()
        return super().autonomousInit()

    
    def testInit(self) -> None:
        return super().testInit()
    
    def testPeriodic(self):
        """ 
        A test routine that runs every 20 ms. Very useful for new methods.
        """
        #Calibration testing
        #self.elevator.manualControl(self.applyDeadband(self.controller.getLeftY()))
        #self.algaeArm.manualControl(self.applyDeadband(self.controller.getRightY()))
        #self.climber.manualControl(self.applyDeadband(self.controller.getRightY()))
            
        return super().testPeriodic()

    def teleopPeriodic(self) -> None:        
        """
        Manual control mode that runs every 20 ms. 
        """
        #Drive Controls
        self.xSpeed = self.applyDeadband(self.controller.getLeftY()) * 4
        self.ySpeed = self.applyDeadband(self.controller.getLeftX()) * 4
        
        #Auto aim code
        if (self.limelight.targetCheck() and self.controller.getLeftTriggerAxis() == 1):
            self.led.green()
            self.rot = self.limelight.aim()
        else:
            self.led.greenBreathing()
            self.rot = self.applyDeadband(self.controller.getRightX()) * 4
        
        if (self.controller.getRightBumper()):
            self.drivetrain.reset()


        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()

        #Switch april tag tracking offsets
        if (self.controller.getLeftBumper()):
            self.limelight.aprilTagPipelineLeft()

        if (self.controller.getRightBumper()):
            self.limelight.aprilTagPipelineRight()

        #switch to algae Tracking pipeline
        if (self.controller.getAButton()):
            self.limelight.aiPipeline()

            
    def manualDrive(self) -> None:
        """ 
        passes the controller speeds to the drivetrain for movement.
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.drive(speeds)

        
    def getTemps(self):
        """ 
        Get temps for all devices on the robot.
        """
        wpilib.SmartDashboard.putNumberArray("drivetrain device temps", self.driveTrainDeviceTemps)
        
    def pdpStats(self):
        """ 
        Gets current robot voltage, total power draw, and the total current draw.
        """
        wpilib.SmartDashboard.putNumber("Robot Voltage", self.pdp.getVoltage())
        wpilib.SmartDashboard.putNumber("Robot Power Draw", self.pdp.getTotalPower())
        wpilib.SmartDashboard.putNumber("Robot Current Draw", self.pdp.getTotalCurrent())
        
    def getBatteryVoltage(self):
        wpilib.SmartDashboard.putNumber("Battery Voltage", wpilib.RobotController.getBatteryVoltage())

    def getMatchTime(self):
        wpilib.SmartDashboard.putNumber("Match Time",  self.timer.getMatchTime())
        #print(self.timer.getMatchTime())
            