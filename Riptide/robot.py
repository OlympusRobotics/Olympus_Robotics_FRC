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
from Subsystems.Limelight4 import limelight4
from Subsystems.Limelight2 import limelight2
from Subsystems.LED import led
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from Subsystems.elevator import Elevator
from Subsystems.AlgaeArm import algaeArm
from Subsystems.AlgaeRemover import algaeRemover
#from Subsystems.Climber import climber

#initalizes the subsystems outside of the class to avoid multiple instances of the subsystems being created while using the test command.
drivetrain = Drivetrain()
elevator = Elevator()
limelight4 = limelight4()
algaeArm = algaeArm()
algaeRemover = algaeRemover()

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
        self.driverController = wpilib.XboxController(0)
        self.operatorController = wpilib.XboxController(1)

        self.elevator = elevator
        self.drivetrain = drivetrain
        self.limelight = limelight4
        self.algaeArm = algaeArm
        self.algaeRemover = algaeRemover
        #self.climber = climber
        self.led = led()
        self.DS = wpilib.DriverStation
        self.irSensor = wpilib.DigitalInput(0)
        
        #autos
        self.test = "Test"
        self.square = "Square"

        self.chooser = wpilib.SendableChooser()

        self.chooser.setDefaultOption("Test1", self.test)
        self.chooser.addOption("Test2", self.square)
        self.chooser.addOption("Test3", None)
        self.chooser.addOption("Test4", None)
        self.chooser.addOption("Test5", None)

        wpilib.SmartDashboard.putData("Auto Options", self.chooser)
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        #SwerveModule states are configured to publish to networkTables for logging and tracking. 
        swerveStates = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.swervePublish = swerveStates.publish()
        
        #Initializes a field for odometry tracking.
        self.field = wpilib.Field2d()

        #Sends target data to the dashboard.
        wpilib.SmartDashboard.putBoolean("AprilTag Target", self.limelight.targetCheck())

        
        #Commands
        self.algaeArmHomePosition = commands2.InstantCommand(self.algaeArm.setHomePosition, self)
        self.algaeArmIntakePosition = commands2.InstantCommand(self.algaeArm.setIntakePosition, self)
        self.algaeEjectPosition = commands2.InstantCommand(self.algaeArm.setEjectPosition, self)

        self.elevatorReturnHome = commands2.InstantCommand(self.elevator.setHome, self)
        self.elevatorIntakePosition = commands2.InstantCommand(self.elevator.setintake, self)
        self.elevatorL1 = commands2.InstantCommand(self.elevator.setL1, self)
        self.elevatorL2 = commands2.InstantCommand(self.elevator.setL2, self)
        self.elevatorL3 = commands2.InstantCommand(self.elevator.setL3, self)

        self.setAlgaeRemoverHomePosition = commands2.InstantCommand(self.algaeRemover.setHomePosition, self)
        self.setAlgaeRemoverReadyPosition = commands2.InstantCommand(self.algaeRemover.setReadyPosition, self)
        self.setAlgaeRemoverPosition1 = commands2.InstantCommand(self.algaeRemover.setPostion1, self)
        self.setAlgaeRemoverPosition2 = commands2.InstantCommand(self.algaeRemover.setPostion2, self)

        self.algaeEjectReturnHome = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.algaeArm.algaeEject, self),
            commands2.WaitCommand(1),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor, self),
            commands2.InstantCommand(self.algaeArm.setHomePosition, self)
        )

        self.algaeIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.algaeArm.setIntakePosition, self),
            commands2.InstantCommand(self.algaeArm.intake, self),
            commands2.WaitCommand(0.4),
            commands2.WaitUntilCommand(condition=self.algaeArm.algaeCheck),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor, self),
            commands2.InstantCommand(self.algaeArm.setEjectPosition, self)
            
        )
                
        self.intakeCoralTransferL1 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(0.3),
            #commands2.WaitUntilCommand(condition=self.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL1, self)
        )
        
        self.intakeCoralTransferL2 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(0.3),
            #commands2.WaitUntilCommand(condition=self.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL2, self)
        )
        
        self.intakeCoralTransferL3 = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(0.3),
            #commands2.WaitUntilCommand(condition=self.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setL3, self)
        )
        
        self.coralEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(2),
            #commands2.WaitUntilCommand(condition=self.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(.5)
            #commands2.InstantCommand(self.elevator.setHome, self)
        ) 

        self.coralUnstuck = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.coralUnstuck, self),
            commands2.WaitCommand(0.3),
            commands2.InstantCommand(self.elevator.flyWheelStop, self)
        )

        self.coralIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(1),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.WaitCommand(0.1),
            commands2.InstantCommand(self.elevator.flyWheelStop, self)
        )

        self.elevatorResetPosition = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitCommand(.1),
            #commands2.WaitUntilCommand(condition=self.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.WaitCommand(0.2),
            commands2.InstantCommand(self.elevator.setHome, self),
            commands2.InstantCommand(self.algaeRemover.setHomePosition, self)
        )

        self.climbFinal = commands2.InstantCommand(self.algaeArm.setHomePosition).alongWith(commands2.InstantCommand(self.elevator.setHome).alongWith(commands2.InstantCommand(self.algaeRemover.setHomePosition)))

        #sending commands above to Pathplanner
        NamedCommands.registerCommand("AlgaeEjectPosition", self.algaeEjectPosition)
        NamedCommands.registerCommand("AlgaeEjectReturnHome", self.algaeEjectReturnHome)
        NamedCommands.registerCommand("AlgaeArmIntakePosition", self.algaeArmIntakePosition)
        NamedCommands.registerCommand("AlgaeArmHomePosition", self.algaeArmHomePosition)

        NamedCommands.registerCommand("setAlgaeRemoverHomePosition", self.setAlgaeRemoverHomePosition)
        NamedCommands.registerCommand("setAlgaeRemoverReadyPosition", self.setAlgaeRemoverReadyPosition)
        NamedCommands.registerCommand("setAlgaeRemoverPosition1", self.setAlgaeRemoverPosition1)
        NamedCommands.registerCommand("setAlgaeRemoverPosition2", self.setAlgaeRemoverPosition2)
        
        NamedCommands.registerCommand("elevatorReturnHome", self.elevatorReturnHome)
        NamedCommands.registerCommand("ElevatorIntake", self.elevatorIntakePosition)
        NamedCommands.registerCommand("elevatorL1", self.elevatorL1)
        NamedCommands.registerCommand("elevatorL2", self.elevatorL2)
        NamedCommands.registerCommand("elevatorL3", self.elevatorL3)
        
        NamedCommands.registerCommand("IntakeCoralTransferL1", self.intakeCoralTransferL1)
        NamedCommands.registerCommand("IntakeCoralTransferL2", self.intakeCoralTransferL2)
        NamedCommands.registerCommand("IntakeCoralTransferL3", self.intakeCoralTransferL3)
        NamedCommands.registerCommand("CoralEject", self.coralEject)
        NamedCommands.registerCommand("ClimbFinal", self.climbFinal)
        NamedCommands.registerCommand("CoralUnstuck", self.coralUnstuck)
        NamedCommands.registerCommand("CoralIntake", self.coralIntake)

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
        
        #Adds the robot pose to the field that was constructed in robotInit.
        wpilib.SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.drivetrain.odometry.getPose())

        wpilib.SmartDashboard.putBoolean("AprilTag Target", self.limelight.targetCheck())
        wpilib.SmartDashboard.putBoolean("Coral loaded", self.coralCheck())
        
        self.getBatteryVoltage()
       
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
        self.autoSelected = self.chooser.getSelected()
        auto = PathPlannerAuto(self.autoSelected)

        return auto
    
    def autonomousInit(self):
        """ 
        Runs the auto selection.
        """
        self.autoCommand = self.getAutoCommand()

        self.autoCommand.schedule()
        return super().autonomousInit()

    
    def testInit(self) -> None:
        #self.coralIntake.schedule()
        #self.algaeIntake.schedule()
        #self.elevatorIntakePosition.schedule()
        #self.coralIntake.schedule()
        #self.elevatorL1.schedule()

        return super().testInit()
    
    def testPeriodic(self):
        """ 
        A test routine that runs every 20 ms. Very useful for new methods.
        """
            
        """if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()
"""
        if (self.driverController.getAButton()):
            self.setAlgaeRemoverReadyPosition.schedule()

        if (self.driverController.getYButton()):
            self.setAlgaeRemoverPosition2.schedule()

        if (self.driverController.getBButton()):
            self.coralIntake.schedule()

        if (self.driverController.getXButton()):
            self.elevatorIntakePosition.schedule()

        if (self.driverController.getRightBumper()):
            self.coralEject.schedule()

        

        wpilib.SmartDashboard.getNumber("Elevator Position", self.elevator.elevatorEncoder1.getPosition())
        wpilib.SmartDashboard.getNumber("Algae Arm Position", self.algaeArm.armRotationEncoder.getPosition())


        wpilib.SmartDashboard.putBoolean("Apriltag Target", self.limelight.targetCheck())
            
            
        return super().testPeriodic()

    def teleopPeriodic(self) -> None:        
        """
        Manual control mode that runs every 20 ms. 
        """

        #Driver Controls
        if (self.driverController.getLeftBumperButton()):
            self.limelight.aprilTagPipelineLeft()

        if (self.driverController.getRightBumperButton()):
            self.limelight.aprilTagPipelineRight()

        if (self.driverController.getLeftTriggerAxis() > 0.5):

            if (self.limelight.targetCheck()):
                self.led.green()
                self.rot = self.limelight.aim()
            
            else:
                self.led.greenBlink()

        else:
            self.led.rainbow()

        if (self.driverController.getXButton()): 
            self.led.redBlink()
            speeds = self.limelight.aimAndRange()

            self.xSpeed = speeds[0] * 4
            self.ySpeed = speeds[1] * 4
            self.rot = speeds[2] * 4
        
        else:
            self.led.rainbow()

        self.xSpeed = self.applyDeadband(self.driverController.getLeftY()) * 4
        self.ySpeed = self.applyDeadband(self.driverController.getLeftX()) * 4
        self.rot = self.applyDeadband(self.driverController.getRightX()) * 4
            
        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()

        if (self.driverController.getRightTriggerAxis() >= 0.5):
            self.elevator.flyWheelSpin()

        #Operator Controls
        if (self.operatorController.getLeftTriggerAxis() >= 0.5):
            self.coralIntake.schedule()

        if (self.operatorController.getRightTriggerAxis() >= 0.5):
            self.coralEject.schedule()

        if (self.operatorController.getLeftBumper()):
            self.algaeIntake.schedule()

        if (self.operatorController.getRightBumper()):
            self.algaeEjectReturnHome.schedule()

        if (self.operatorController.getYButton()):
            self.algaeArmIntakePosition.schedule()

        if (self.operatorController.getAButton()):
            self.algaeArm.setHomePosition()

        if (self.operatorController.getXButton()):
            self.elevator.setHome()

        if (self.operatorController.getBButton()):
            self.coralUnstuck.schedule()


        if (self.operatorController.getPOV() == 270):
            self.elevator.setL3()

        if (self.operatorController.getPOV() == 90):
            self.elevator.setL2()

        if (self.operatorController.getPOV() == 180):
            self.elevator.setL1()

        if (self.operatorController.getStartButton()):
            self.climbFinal.schedule()

        if (self.operatorController.getBackButton()):
            self.elevatorResetPosition.schedule()

        if ((self.operatorController.getPOV() == 0) and (self.operatorController.getXButton())):
            self.setAlgaeRemoverPosition1.schedule()

        if ((self.operatorController.getPOV() == 0) and (self.operatorController.getBButton())):
            self.setAlgaeRemoverPosition2.schedule()

        
    def coralCheck(self):
        return not self.irSensor.get()

    def manualDrive(self) -> None:
        """ 
        passes the controller speeds to the drivetrain for movement.
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.drive(speeds)

        
    def robotIsTooHot(self):
        """ 
        Monitors the different subsystems for overheating. 
        """
        if (self.drivetrain.drivetrainIsTooHot()):
            for i in range(100):
                print("--------DRIVETRAIN IS TOO HOT--------")
                
        if (self.elevator.isTooHot()):
            for i in range(100):
                print("--------ELEVATOR IS TOO HOT--------")
                
        if (self.algaeArm.isTooHot()):
            for i in range(100):
                print("--------ALGAE ARM IS TOO HOT--------")
                
        if (self.algaeRemover.isTooHot()):
            for i in range(100):
                print("--------ALGAE REMOVER IS TOO HOT--------")
        
    def getBatteryVoltage(self):
        """ 
        Gets the battery voltage and sends it to the dashboard.
        """
        wpilib.SmartDashboard.putNumber("Battery Voltage", wpilib.RobotController.getBatteryVoltage())

            