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
limelight2 = limelight2()
algaeArm = algaeArm()
algaeRemover = algaeRemover()

#climber = climber()

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
        self.limelightAlgae = limelight4
        self.limelightAprilTag = limelight2
        self.algaeArm = algaeArm
        self.algaeRemover = algaeRemover
        #self.climber = climber
        self.led = led()
        self.DS = wpilib.DriverStation
        
        self.aimMode = "None"

        #autos
        self.test = "Test"
        self.square = "Square"

        self.chooser = wpilib.SendableChooser()

        self.chooser.addOption("Test2", self.test)
        self.chooser.addOption("Test3", self.square)
        self.chooser.addOption("Test4", None)
        self.chooser.addOption("Test5", None)

        self.chooser.setDefaultOption("Test1", None)

        wpilib.SmartDashboard.putData("Auto Options", self.chooser)
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        #SwerveModule states are configured to publish to networkTables for logging and tracking. 
        swerveStates = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.swervePublish = swerveStates.publish()
        
        #Initializes a field for odometry tracking.
        self.field = wpilib.Field2d()

        
        #Commands
        self.algaeArmHomePosition = commands2.InstantCommand(self.algaeArm.setHomePosition, self)
        self.algaeArmIntakePosition = commands2.InstantCommand(self.algaeArm.setIntakePosition, self)

        self.elevatorReturnHome = commands2.InstantCommand(self.elevator.setHome(), self)
        self.elevatorL1 = commands2.InstantCommand(self.elevator.setL1(), self)
        self.elevatorL2 = commands2.InstantCommand(self.elevator.setL2(), self)
        self.elevatorL3 = commands2.InstantCommand(self.elevator.setL3(), self)

        self.setAlgaeRemoverHomePosition = commands2.InstantCommand(self.algaeRemover.setHomePosition, self)
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
        
        self.coralEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self),
            commands2.InstantCommand(self.elevator.setHome, self)
        ) 

        self.coralUnstuck = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.coralUnstuck, self),
            commands2.WaitCommand(0.3),
            commands2.InstantCommand(self.elevator.flyWheelStop, self)
        )

        self.coralIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin, self),
            commands2.WaitUntilCommand(condition=self.elevator.coralCheck),
            commands2.InstantCommand(self.elevator.flyWheelStop, self)
        )

        self.climbFinal = commands2.InstantCommand(self.algaeArm.setHomePosition).alongWith(commands2.InstantCommand(self.elevator.setHome))

        #sending commands above to Pathplanner
        NamedCommands.registerCommand("AlgaeEjectReturnHome", self.algaeEjectReturnHome)
        NamedCommands.registerCommand("AlgaeArmIntakePosition", self.algaeArmIntakePosition)
        NamedCommands.registerCommand("AlgaeArmHomePosition", self.algaeArmHomePosition)

        NamedCommands.registerCommand("setAlgaeRemoverHomePosition", self.setAlgaeRemoverHomePosition)
        NamedCommands.registerCommand("setAlgaeRemoverPosition1", self.setAlgaeRemoverPosition1)
        NamedCommands.registerCommand("setAlgaeRemoverPosition2", self.setAlgaeRemoverPosition2)
        
        NamedCommands.registerCommand("elevatorReturnHome", self.elevatorReturnHome)
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
        
        self.getBatteryVoltage()
        #self.robotIsTooHot()
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
        self.autoSelected = self.chooser.getSelected()
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
        self.climbFinal.schedule()
        return super().testInit()
    
    def testPeriodic(self):
        """ 
        A test routine that runs every 20 ms. Very useful for new methods.
        """

        #Calibration testing
        """ self.elevator.manualControl(self.applyDeadband(self.driverController.getLeftY()))
        self.algaeArm.manualControl(self.applyDeadband(self.operatorController.getLeftY()))

        wpilib.SmartDashboard.getNumber("Elevator Position", self.elevator.elevatorEncoder1.getPosition())
        wpilib.SmartDashboard.getNumber("Algae Arm Position", self.algaeArm.armRotationEncoder.getPosition()) """

        #wpilib.SmartDashboard.putNumberArray("Limelight 3d values",self.limelightAprilTag.aimAndRange())

        #Switch april tag tracking offsets

        wpilib.SmartDashboard.putString("Aim Mode", self.aimMode)
        wpilib.SmartDashboard.putBoolean("Apriltag Target", self.limelightAprilTag.targetCheck())
        wpilib.SmartDashboard.putBoolean("Algae Target", self.limelightAlgae.targetCheck())

        #self.led.rainbow()


            


            
        return super().testPeriodic()

    def teleopPeriodic(self) -> None:        
        """
        Manual control mode that runs every 20 ms. 
        """
        #Drive Controls
        self.xSpeed = self.applyDeadband(self.driverController.getLeftY()) * 4
        self.ySpeed = self.applyDeadband(self.driverController.getLeftX()) * 4
        
        if (self.driverController.getLeftBumperButton()):
            self.limelightAprilTag.aprilTagPipelineLeft()
            self.aimMode = "AprilTag"

        if (self.driverController.getRightBumperButton()):
            self.limelightAprilTag.aprilTagPipelineRight()
            self.aimMode = "AprilTag"
            
        if (self.driverController.getYButton()):
            self.limelightAlgae.aiPipeline()
            self.aimMode = "Algae"

        if (self.driverController.getLeftTriggerAxis() > 0.5):

            if (self.limelightAprilTag.targetCheck()):
                self.led.green()
                self.rot = self.limelightAprilTag.aim()
            
            elif (self.limelightAlgae.targetCheck()):
                self.led.green()
                self.rot = self.limelightAlgae.aim()

            else:
                self.led.greenBlink()

        if (self.driverController.getXButton()):
            self.led.redBlink()
            speeds = self.limelightAprilTag.aimAndRange()

            self.xSpeed = speeds[0] * 4
            self.ySpeed = speeds[1] * 4
            self.rot = speeds[2] * 4
            
            
        #Auto aim code
        if (self.aimMode == "Apriltag" and self.limelightAprilTag.targetCheck() and  self.driverController.getLeftTriggerAxis() == 1):
            self.led.green()
            self.rot = self.limelightAprilTag.aim()
        
        elif (self.aimMode == "Algae" and self.limelightAlgae.targetCheck() and self.driverController.getLeftTriggerAxis() == 1):
            self.led.green()
            self.rot = self.limelightAlgae.aim()

        elif (self.aimMode == "Apriltag"):
            if (not self.limelightAprilTag.targetCheck() and self.driverController.getLeftTriggerAxis() == 1):
                self.led.greenBreathing()

        elif (self.aimMode == "Algae"):
            if (not self.limelightAlgae.targetCheck() and self.driverController.getLeftTriggerAxis() == 1):
                self.led.greenBreathing()
            
        else:
            self.rot = self.applyDeadband(self.driverController.getRightX()) * 4
            
        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()

        if (self.driverController.getRightTriggerAxis() == 1):
            self.elevator.flyWheelSpin()

            
        
        #Operator Controls
        if (self.operatorController.getLeftTriggerAxis() == 1):
            self.coralIntake

        if (self.operatorController.getRightTriggerAxis() == 1):
            self.coralEject

        if (self.operatorController.getLeftBumper()):
            self.algaeIntake

        if (self.operatorController.getRightBumper()):
            self.algaeEjectReturnHome

        if (self.operatorController.getYButton()):
            self.algaeArm.intakePosition()

        if (self.operatorController.getAButton()):
            self.algaeArm.setHomePosition()

        if (self.operatorController.getXButton()):
            self.elevator.setHomePosition()

        if (self.operatorController.getBButton()):
            self.coralUnstuck

        if (self.operatorController.getLeftStickButton()):
            self.elevator.manualControl(self.operatorController.getLeftY())

        if (self.operatorController.getPOV() == 270):
            self.elevatorL3

        if (self.operatorController.getPOV() == 90):
            self.elevatorL2

        if (self.operatorController.getPOV() == 180):
            self.elevatorL1


        #Add Fang Subsystem

    def manualDrive(self) -> None:
        """ 
        passes the controller speeds to the drivetrain for movement.
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.drive(speeds)

        
    def robotIsTooHot(self):
        if (self.drivetrain.drivetrainIsTooHot()):
            for i in range(100):
                print("--------DRIVETRAIN IS TOO HOT--------")
                
        elif (self.elevator.isTooHot()):
            for i in range(100):
                print("--------ELEVATOR IS TOO HOT--------")
                
        elif (self.algaeArm.isTooHot()):
            for i in range(100):
                print("--------ALGAE ARM IS TOO HOT--------")
                
        
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
            