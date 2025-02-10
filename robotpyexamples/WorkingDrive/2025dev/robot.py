
import commands2
import ntcore
import pathplannerlib.auto
import pathplannerlib.config
import wpilib
import wpilib.simulation
import phoenix6
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import Subsystems.drivetrain as drivetrain
import pathplannerlib
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import SmartDashboard

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.drivetrain = drivetrain.Drivetrain()
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()
        # Start publishing an array of module states with the "/SwerveStates" key
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.pub = topic.publish()

        self.timer = wpilib.Timer()
        self.timer.start()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.



    def robotPeriodic(self): 
        self.drivetrain.updateOdometry()        
        wpilib.SmartDashboard.putNumber("FLD Temp", self.drivetrain.flSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("FLR Temp", self.drivetrain.flSM.rotationMotor.getMotorTemperature())
        
        wpilib.SmartDashboard.putNumber("FRD Temp", self.drivetrain.frSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("FRR Temp", self.drivetrain.frSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BLD Temp", self.drivetrain.blSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("BLR Temp", self.drivetrain.blSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BRD Temp", self.drivetrain.brSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("BRR Temp", self.drivetrain.brSM.rotationMotor.getMotorTemperature())


        

        return super().robotPeriodic()

    """ def autonomousPeriodic(self) -> None:
        self.drivetrain.updateOdometry() """

    def applyDeadband(self, value, deadband=0.15):
        return value if abs(value) > deadband else 0

    def teleopPeriodic(self) -> None:      
        self.pub.set([self.drivetrain.flSM.getState(),self.drivetrain.frSM.getState(),self.drivetrain.blSM.getState(),self.drivetrain.frSM.getState()])
        
        self.xSpeed = self.applyDeadband(self.controller.getLeftY())
        self.ySpeed = self.applyDeadband(self.controller.getLeftX())
        self.rot = self.applyDeadband(self.controller.getRightX())

        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            self.manualDrive()
    
    def manualDrive(self) -> None:
        """
        Field Oriented Drive
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-self.xSpeed, -self.ySpeed, -self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.driveFO(speeds)

    def autoDrive(self) -> None:
        """
        Robot Oriented Drive
        """
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(-self.xSpeed, -self.ySpeed, -self.rot, self.drivetrain.gyro.getRotation2d())
        self.drivetrain.driveRO(speeds)
