
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
import elasticlib
from Subsystems.drivetrain import Drivetrain
from Subsystems.Limelight import limelight
from Subsystems.LED import led
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from Subsystems.elevator import Elevator

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
        self.led = led()
        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)

        #Auto selection
        self.square = "Test"
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        #SwerveModule states are configured to publish to networkTables for logging and tracking. 
        swerveStates = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.swervePublish = swerveStates.publish()
        
        #Initializes a field for odometry tracking.
        self.field = wpilib.Field2d()

        #Creates and starts a timer object.
        self.timer = wpilib.Timer()
        self.timer.start()
         
        self.testNoti = elasticlib.Notification(elasticlib.NotificationLevel.INFO, "test", "test")
        self.climberTempWarning = elasticlib.Notification(elasticlib.NotificationLevel.WARNING, "Climber Motor Temps", "One or more climber motor temperatures are getting too hot. Please disable and shutdown before any damage occurs.")
        self.elevatorTempWarning = elasticlib.Notification(elasticlib.NotificationLevel.WARNING, "Elevator Motor Temps", "One or more elevator motor temperatures are getting too hot. Please disable and shutdown before any damage occurs.")
        self.limelightTempWarning = elasticlib.Notification(elasticlib.NotificationLevel.WARNING, "Limelight Temp", "Limelight is getting to hot. Please disable and shutdown before any damage occurs.")
        self.drivetrainTempWarning = elasticlib.Notification(elasticlib.NotificationLevel.WARNING, "device Temp", "one or more drivetrain device temperatyres are way too hot. Please disable and shutdown before any damage occurs.")
        self.algaeIntakeTempWarning = elasticlib.Notification(elasticlib.NotificationLevel.WARNING, "Climber Motor Temps", "One or more climber motor temperatures are getting too hot. Please disable and shutdown before any damage occurs.")
 
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
      
        #Publishes the motor temps on Smart Dashboard.
        self.getTemps()
        self.DrivetrainTempCheck()
        self.limelightTempCheck()
        
        #Adds the robot pose to the field that was constructed in robotInit.
        wpilib.SmartDashboard.putData("Field", self.field)
        self.field.setRobotPose(self.drivetrain.odometry.getPose())
       

        return super().robotPeriodic()

    def applyDeadband(self, value, deadband=0.08):
        """ 
        Applys a deadband to stop controller drifting.
        """
        return value if abs(value) > deadband else 0
    
    def testInit(self) -> None:
        elasticlib.send_notification(self.testNoti)
        return super().testInit()
    
    def testPeriodic(self):
        """ 
        A test routine that runs every 20 ms. Very useful for new methods.
        """
        self.elevator.manualControl(self.applyDeadband(self.controller.getLeftY()) / 2)
        print(f"ENC 10 Value: {self.elevator.elevatorEncoder1.getPosition()} | ENC 11 Value: { self.elevator.elevatorEncoder2.getPosition()}")
        
        self.led.white()
            
        return super().testPeriodic()

    def teleopPeriodic(self) -> None:        
        """
        Manual control mode that runs every 20 ms. 
        """
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
        
    def DrivetrainTempCheck(self):
        """ 
        Checks if the drivetrain device temps are way too hot (90c or higher) and sends a warning to the driver and operator to disable and shutdown the robot before any damage occurs. 
        """
        if (self.drivetrain.drivetrainIsTooHot()):
            elasticlib.send_notification(self.drivetrainTempWarning)
    
    def limelightTempCheck(self):
        """ 
        Checks both the cpu and general device temps on the limelight and warns the driver and operator if the temps are too high. 
        """
        if (self.limelight.isTooHot()):
            elasticlib.send_notification(self.limelightTempWarning)
            
        
        
