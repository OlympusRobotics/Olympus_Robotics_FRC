
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

        self.square = "Test"
        
        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)

        commands2.InstantCommand(self.drivetrain.stopDrivetrain(), self)
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()

        # Start publishing an array of module states with the "/SwerveStates" key
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.pub = topic.publish()
        
        self.field = wpilib.Field2d()
        # Do this in either robot or subsystem init

        self.timer = wpilib.Timer()
        self.timer.start()
 
    def getAutoCommand(self):
        self.autoSelected = self.square
        auto = PathPlannerAuto(self.autoSelected)

        return auto
    
    def autonomousInit(self):
        self.autoCommand = PathPlannerAuto("Test")

        self.autoCommand.schedule()
        return super().autonomousInit()
        
    def autonomousPeriodic(self):
        #self.drivetrain.updateOdometry()
        return super().autonomousPeriodic()


    def robotPeriodic(self): 
        self.drivetrain.updateOdometry()  
        self.pub.set([self.drivetrain.flSM.getState(),self.drivetrain.frSM.getState(),self.drivetrain.blSM.getState(),self.drivetrain.brSM.getState()])    
      
        self.getTemps()
        self.getCurrents()
        
        wpilib.SmartDashboard.putData("Field", self.field)
        # Do this in either robot periodic or subsystem periodic
        self.field.setRobotPose(self.drivetrain.odometry.getPose())
        #self.getPDPStats()
       

        return super().robotPeriodic()

    def applyDeadband(self, value, deadband=0.08):
        return value if abs(value) > deadband else 0
    
    def testInit(self):
        return super().testInit()
    
    def testPeriodic(self):
        """ self.elevator.manualControl(self.applyDeadband(-self.controller.getLeftY()) / 2)

        if (self.limelight.targetCheck()):
            self.led.green()
        else:
            self.led.red() """
        self.led.white()
            
        return super().testPeriodic()

    def teleopPeriodic(self) -> None:        
        
        self.xSpeed = self.applyDeadband(self.controller.getLeftY()) * 4
        self.ySpeed = self.applyDeadband(self.controller.getLeftX()) * 4
        

        if (self.limelight.aim() > 0 and self.controller.getLeftTriggerAxis() >= 1):
            self.rot = self.limelight.aim()

        else:
            self.rot = self.applyDeadband(self.controller.getRightX()) * 4
        
        if (self.controller.getRightBumper()):
            self.drivetrain.reset()


        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()

            
    def manualDrive(self) -> None:
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-self.xSpeed, -self.ySpeed, -self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.drive(speeds)

    def autoDrive(self) -> None:
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-self.xSpeed, -self.ySpeed, -self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.drive(speeds)
        
    def getTemps(self):
        wpilib.SmartDashboard.putNumber("FLD Temp", self.drivetrain.flSM.driveMotor.get_device_temp().value_as_double)
        wpilib.SmartDashboard.putNumber("FLR Temp", self.drivetrain.flSM.rotationMotor.getMotorTemperature())
        
        wpilib.SmartDashboard.putNumber("FRD Temp", self.drivetrain.frSM.driveMotor.get_device_temp().value_as_double)
        wpilib.SmartDashboard.putNumber("FRR Temp", self.drivetrain.frSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BLD Temp", self.drivetrain.blSM.driveMotor.get_device_temp().value_as_double)
        wpilib.SmartDashboard.putNumber("BLR Temp", self.drivetrain.blSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BRD Temp", self.drivetrain.brSM.driveMotor.get_device_temp().value_as_double)
        wpilib.SmartDashboard.putNumber("BRR Temp", self.drivetrain.brSM.rotationMotor.getMotorTemperature())
        
    def getCurrents(self):
        wpilib.SmartDashboard.putNumber("FLD current", self.drivetrain.flSM.driveMotor.get_stator_current().value_as_double)
        wpilib.SmartDashboard.putNumber("FLR current", self.drivetrain.flSM.rotationMotor.getOutputCurrent())
        
        wpilib.SmartDashboard.putNumber("FRD current", self.drivetrain.frSM.driveMotor.get_stator_current().value_as_double)
        wpilib.SmartDashboard.putNumber("FRR current", self.drivetrain.frSM.rotationMotor.getOutputCurrent())

        wpilib.SmartDashboard.putNumber("BLD current", self.drivetrain.blSM.driveMotor.get_stator_current().value_as_double)
        wpilib.SmartDashboard.putNumber("BLR current", self.drivetrain.blSM.rotationMotor.getOutputCurrent())

        wpilib.SmartDashboard.putNumber("BRD current", self.drivetrain.brSM.driveMotor.get_stator_current().value_as_double)
        wpilib.SmartDashboard.putNumber("BRR current", self.drivetrain.brSM.rotationMotor.getOutputCurrent())
        
    def getPDPStats(self):
        wpilib.SmartDashboard.putNumber("Total Current Draw", self.pdp.getTotalCurrent())
        wpilib.SmartDashboard.putNumber("Total Power Draw", self.pdp.getTotalPower())
