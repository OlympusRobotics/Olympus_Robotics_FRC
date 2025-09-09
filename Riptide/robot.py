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
from Subsystems.LED import led
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from Subsystems.elevator import Elevator
from Subsystems.AlgaeArm import algaeArm
from Subsystems.AlgaeRemover import algaeRemover
from cscore import CameraServer as CS
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
        PIDConstants(7, 0.0, 0.0),
        PIDConstants(5, 0.0, 0.0),
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

        self.irSensor = wpilib.DigitalInput(1)
        self.irSensor2 = wpilib.DigitalInput(3)
        CS.startAutomaticCapture()

        #leds
        self.led = led()

        self.DS = wpilib.DriverStation
        
        #autos
        self.test = "2 Meter Auto"
        self.centerLineScore = "Center to H Score"
        self.barge1ToPark = "Barge 1 to Park"
        self.flower = "Algae Pull Out"
        self.curve = "Curve"
        self.halfField = "Half Field Test"
        self.threeCoral = "Rm Alg +~ 2 cr"
        self.Groundprocess = "Ground process"
        self.L3oneCoral = "1CoralL3"

        self.chooser = wpilib.SendableChooser()

        self.chooser.setDefaultOption("Rm Alg +~ 2 cr", self.threeCoral)
        self.chooser.addOption("Center to H Score", self.centerLineScore)
        self.chooser.addOption("Barge 1 Park", self.barge1ToPark)
        self.chooser.addOption("Algae Pull Out", self.flower)
        self.chooser.addOption("Ground process", self.Groundprocess)
        self.chooser.addOption("One coral on L3", self.L3oneCoral)

        wpilib.SmartDashboard.putData("Auto Options", self.chooser)

        #Subsystem Status
        self.elevatorPosition = ""
        self.algaeArmPosition = ""
        self.algaeRemoverPosition = ""
        
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
        """ self.algaeArmHomePosition = commands2.InstantCommand(lambda: self.algaeArm.setPosition(NewPosition=self.algaeArm.homePosition), self)
        self.algaeArmIntakePosition = commands2.InstantCommand(lambda: self.algaeArm.setPosition(NewPosition=self.algaeArm.intakePosition), self)
        self.algaeEjectPosition = commands2.InstantCommand(lambda: self.algaeArm.setPosition(NewPosition=self.algaeArm.algaeEjectPosition), self) """

        self.elevatorReturnHome = commands2.InstantCommand(self.elevator.setHome)
        self.elevatorIntakePosition = commands2.InstantCommand(self.elevator.setIntake)
        self.elevatorL1 = commands2.InstantCommand(self.elevator.setL1)
        self.elevatorL2 = commands2.InstantCommand(self.elevator.setL2)
        self.elevatorL3 = commands2.InstantCommand(self.elevator.setL3)

        self.setAlgaeRemoverHomePosition = commands2.InstantCommand(self.algaeRemover.setHomePosition)
        self.setAlgaeRemoverReadyPosition = commands2.InstantCommand(self.algaeRemover.setReadyPosition)
        self.setAlgaeRemoverPosition1 = commands2.InstantCommand(self.algaeRemover.setPostion1)
        self.setAlgaeRemoverPosition2 = commands2.InstantCommand(self.algaeRemover.setPostion2)

        self.algaeEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.algaeArm.algaeEject),
            commands2.WaitCommand(1),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor),
            commands2.InstantCommand(self.setAlgaeIntakeHomePosition)
        )

        self.algaeIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.setAlgaeIntakeFeedPosition),
            commands2.InstantCommand(self.algaeArm.intake),
            commands2.WaitCommand(1),
            commands2.WaitUntilCommand(condition=self.operatorController.getLeftBumperButtonReleased),
            commands2.WaitCommand(0.4),
            commands2.InstantCommand(self.algaeArm.stopIntakeMotor),
            commands2.InstantCommand(self.setAlgaeIntakeEjectPosition)
        )
        
        self.coralEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin),
            commands2.WaitCommand(2),
            commands2.InstantCommand(self.elevator.flyWheelStop),
            commands2.WaitCommand(.5),
            commands2.InstantCommand(self.elevator.setIntake)
        ) 

        self.coralIntake = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.flyWheelSpin),
            commands2.WaitCommand(1),
            commands2.WaitUntilCommand(self.coralCheck),
            commands2.WaitCommand(.06),
            commands2.InstantCommand(self.elevator.flyWheelStop)
        )

        self.coralUnstuck = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.elevator.coralUnstuck),
            commands2.WaitCommand(0.5),
            commands2.InstantCommand(self.elevator.flyWheelStop)
        )
        self.elevator.elevatorEncoder1.setPosition(-41)
        self.elevator.elevatorEncoder2.setPosition(-41)

        self.drivetrainStop = commands2.InstantCommand(self.drivetrain.stopDrivetrain)

        #self.climbFinal = commands2.InstantCommand(self.algaeArm.setHomePosition).alongWith(commands2.InstantCommand(self.elevator.setHome).alongWith(commands2.InstantCommand(self.algaeRemover.setHomePosition)))

        #sending commands above to Pathplanner
        NamedCommands.registerCommand("StopDrivetrain", self.drivetrainStop)

        NamedCommands.registerCommand("setAlgaeRemoverHomePosition", self.setAlgaeRemoverHomePosition)
        NamedCommands.registerCommand("setAlgaeRemoverReadyPosition", self.setAlgaeRemoverReadyPosition)
        NamedCommands.registerCommand("setAlgaeRemoverPosition1", self.setAlgaeRemoverPosition1)
        NamedCommands.registerCommand("setAlgaeRemoverPosition2", self.setAlgaeRemoverPosition2)
        
        NamedCommands.registerCommand("elevatorReturnHome", self.elevatorReturnHome)
        NamedCommands.registerCommand("elevatorIntakePosition", self.elevatorIntakePosition)
        NamedCommands.registerCommand("elevatorL1", self.elevatorL1)
        NamedCommands.registerCommand("elevatorL2", self.elevatorL2)
        NamedCommands.registerCommand("elevatorL3", self.elevatorL3)
        
        NamedCommands.registerCommand("coralEject", self.coralEject)
        NamedCommands.registerCommand("coralIntake", self.coralIntake)
        self.algaeArmPosition = "Home"
        

        #Creates and starts a timer object.
        self.timer = wpilib.Timer()
        self.timer.start()

    def robotPeriodic(self): 
        """ 
        Method that runs every 20ms regardless of what mode the robot is in.
        """
        self.algaeArm.setPosition(NewPosition=self.algaeArmPosition)

        #Always updates the current position and state of the robot and swerve modules respectively.
        self.drivetrain.updateOdometry()  
        self.swervePublish.set([self.drivetrain.flSM.getState(),self.drivetrain.frSM.getState(),self.drivetrain.blSM.getState(),self.drivetrain.brSM.getState()])             
        
        #Adds the robot pose to the field that was constructed in robotInit.
        wpilib.SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.drivetrain.odometry.getPose())

        wpilib.SmartDashboard.putBoolean("AprilTag Target", self.limelight.targetCheck())
        wpilib.SmartDashboard.putNumber("Elv Encoder", self.elevator.elevatorEncoder1.getPosition())
        wpilib.SmartDashboard.putBoolean("ElevatorL1", self.ElevatorL1())
        wpilib.SmartDashboard.putString("Elevator Position", self.elevatorPosition)
        wpilib.SmartDashboard.putString("Algae Arm Position", self.algaeArmPosition)
        wpilib.SmartDashboard.putString("Algae Remover Position", self.algaeRemoverPosition)
        
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
        #self.algaeArm.setHomePosition()
        #self.algaeArm.setEjectPosition()
        #self.setAlgaeRemoverPosition1.schedule()
        return super().testInit()
    
    def testPeriodic(self):
        """ 
        A test routine that runs every 20 ms. Very useful for new methods.
        """
        wpilib.SmartDashboard.putNumber("Algae Arm Actual Position", self.algaeArm.getPosition())
        wpilib.SmartDashboard.putBoolean("Apriltag Target", self.limelight.targetCheck())           
            
        return super().testPeriodic()
    
    def teleopInit(self):
        self.algaeArmPosition = "Home"
        self.elevator.setL1()
        return super().teleopInit()

    def teleopPeriodic(self) -> None:        
        """
        Manual control mode that runs every 20 ms. 
        """

        #Driver Controls
        if (self.operatorController.getPOV() == 0):
            self.elevatorL3.schedule()
            self.elevatorPosition = "L3"

        if (self.operatorController.getPOV() == 90):
            self.elevatorL2.schedule()
            self.elevatorPosition = "L2"

        if (self.operatorController.getPOV() == 180):
            self.elevatorL1.schedule()
            self.elevatorPosition = "L1"
        
        if (self.operatorController.getPOV() == 270):
            self.elevator.setIntake()
            self.elevatorPosition = "Intake"

        

        if (self.driverController.getLeftBumperButton()):
            self.limelight.aprilTagPipelineLeft()

        if (self.driverController.getRightBumperButton()):
            self.limelight.aprilTagPipelineRight()

        if (self.driverController.getLeftTriggerAxis() > 0.5):

            if (self.limelight.targetCheck()):
                self.rot = self.limelight.aim()

            else:
                self.rot = self.applyDeadband(-self.driverController.getRightX()) * 4

        else:
            self.rot = self.applyDeadband(-self.driverController.getRightX()) * 4
            

        if (self.driverController.getXButton()): 
            speeds = self.limelight.aimAndRange()

            self.xSpeed = speeds[0] * 4
            self.ySpeed = speeds[1] * 4
            self.rot = speeds[2] * 4

            wpilib.SmartDashboard.putNumber("xSpeed", self.xSpeed)
            wpilib.SmartDashboard.putNumber("ySpeed", self.ySpeed)
            wpilib.SmartDashboard.putNumber("rot", self.rot)

        else:
            self.xSpeed = self.applyDeadband(-self.driverController.getLeftY()) * 4
            self.ySpeed = self.applyDeadband(-self.driverController.getLeftX()) * 4


        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()

        if (self.driverController.getAButton()):
            self.drivetrain.reset()

        #Operator Controls
        if (self.irSensor2.get()):
            if (-1> self.elevator.elevatorEncoder1.getPosition() or self.elevator.elevatorEncoder1.getPosition() > 1):
                self.elevator.elevatorEncoder1.setPosition(0)
            return

        if (self.operatorController.getRightTriggerAxis() > 0.7):
            self.coralIntake.schedule()

        if (self.operatorController.getStartButton()):
            self.setAlgaeRemoverHomePosition.schedule()
            self.elevatorPosition = "Home"
            
        if (self.operatorController.getBackButton()):
            self.elevator.setHome()
            
        if (self.operatorController.getRightStickButtonPressed()):
            self.setAlgaeRemoverReadyPosition.schedule()
            self.algaeRemoverPosition = "Ready"
        
        if (self.operatorController.getRightY() < -0.75):
            self.setAlgaeRemoverPosition2.schedule()
                   
        if (self.operatorController.getRightY() > 0.75):
            self.setAlgaeRemoverPosition1.schedule()

        """ if (self.operatorController.getAButton()):
            self.setAlgaeIntakeFeedPosition()

        if (self.operatorController.getBButton()):
            self.setAlgaeIntakeEjectPosition()

        if (self.operatorController.getYButton()):
            self.setAlgaeIntakeHomePosition() """

        if (self.operatorController.getLeftBumper()):
            self.algaeIntake.schedule()

        """if (self.operatorController.getBButton()):
            self.coralUnstuck.schedule()"""

        """ if not (self.algaeIntake.isScheduled()):
            if (self.operatorController.getLeftStickButton()):
                self.algaeArm.intake()

            else:
                self.algaeArm.intakeMotor.stopMotor() """
                

        if not (self.coralIntake.isScheduled() or self.coralUnstuck.isScheduled()):
            if (self.operatorController.getLeftTriggerAxis() == 1):
                self.elevator.flyWheelSpin()
                
            else:
                self.elevator.flyWheelStop()


        if (self.operatorController.getRightBumper()):
            self.algaeEject.schedule()

        if (self.operatorController.getStartButton()):
            self.setAlgaeRemoverHomePosition.schedule()
        
        #led effects
        if (self.driverController.getLeftTriggerAxis() > 0.7):
            self.led.greenBreathing()

        elif (self.driverController.getXButton() > 0.7):
            self.led.redBlink()
            
        elif (self.coralCheck()):
            self.led.white()

        else:
            self.led.rainbow()

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
        
    def setAlgaeIntakeHomePosition(self):
        self.algaeArmPosition = "Home"
        
    def setAlgaeIntakeEjectPosition(self):
        self.algaeArmPosition = "Eject"
        
    def setAlgaeIntakeFeedPosition(self):
        self.algaeArmPosition = "Intake"

    def coralCheck(self):
        return not self.irSensor.get()
    def ElevatorL1(self):
        return self.irSensor2.get()

        
