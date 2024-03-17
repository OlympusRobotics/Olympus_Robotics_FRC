import random
import rev
import math
import commands2
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from pathplannerlib.geometry_util import flipFieldPos, flipFieldRotation, flipFieldPose
from wpimath import controller
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from wpilib import DriverStation
from wpilib import SmartDashboard, Field2d
import ntcore
import wpilib
import wpilib.drive
import rev
import ntcore
import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import phoenix6
from pathplannerlib.auto import NamedCommands
import intake

def lratio(angle):
    """converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    return (something/.5)*-math.pi


def ticks2radODOMETRY(something):
    # units are in rotations
    return something * 2* math.pi

def deg2Rot2d(deg) -> Rotation2d:
    yaw = deg/360
    return Rotation2d(yaw * math.pi * 2)


def getSwerveModPos(rotEnc : phoenix6.hardware.CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:
    return SwerveModulePosition(
        (driveEnc.getPosition()/6.75)*math.pi*.1016,
        Rotation2d(ticks2radODOMETRY(rotEnc.get_position().value_as_double))
    )


class DriveTrain(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.intake = intake.Intake()


        # DRIVETRAIN INIT

        self.backLeftRotation = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightRotation = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)

        self.backLeftDrive = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftDrive = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        

        self.frontRightDriveEnc = self.frontRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.backRightDriveEnc = self.backRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.backLeftDriveEnc = self.backLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

        self.FrightEnc = phoenix6.hardware.CANcoder(11) 
        self.FleftEnc = phoenix6.hardware.CANcoder(13)
        self.BrightEnc = phoenix6.hardware.CANcoder(10)
        self.BleftEnc = phoenix6.hardware.CANcoder(12)
        """
                self.FleftEnc = phoenix6.hardware.CANcoder(11) 
                self.BleftEnc = phoenix6.hardware.CANcoder(13)
                self.FrightEnc = phoenix6.hardware.CANcoder(10)
                self.BrightEnc = phoenix6.hardware.CANcoder(12)
        """
        # PID SETUP
        Kp = 4
        self.BleftPID = controller.PIDController(Kp,0,.000)
        self.BleftPID.enableContinuousInput(-.5,.5)
        self.BleftPID.setSetpoint(0.0)
        self.BrightPID = controller.PIDController(Kp,0,.000)
        self.BrightPID.enableContinuousInput(-.5,.5)
        self.BrightPID.setSetpoint(0.0)
        self.FleftPID = controller.PIDController(Kp,0,.000)
        self.FleftPID.enableContinuousInput(-.5,.5)
        self.FleftPID.setSetpoint(0.0)
        self.FrightPID = controller.PIDController(Kp,0,.000)
        self.FrightPID.enableContinuousInput(-.5,.5)
        self.FrightPID.setSetpoint(0.0)




        # GYRO INIT
        self.gyro = phoenix6.hardware.Pigeon2(14)
        self.gyro.set_yaw(0)

        # KINEMATICS
        frontrightlocation = Translation2d(.381, .381) 
        frontleftlocation = Translation2d(.381, -.381) 
        backleftlocation = Translation2d(-.381, -.381)         
        backrightlocation = Translation2d(-.381, .381)    



        self.lastChassisSpeed = ChassisSpeeds(0,0,0)

        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)

            ),
            Pose2d()

        )


        self.shouldUpdateIntakeController = True


        
        







    def resetHarder(self, initialPose = Pose2d()):

        print("CALLED RSETHARDER")
                
        """        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                    initialPose = flipFieldPose(initialPose)
        """
        self.gyro.set_yaw(0)#initialPose.rotation().degrees()

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            deg2Rot2d(self.gyro.get_yaw().value_as_double),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            initialPose
        )

        print(self.getPose())


        

    def getPose(self):
        nonYPose = self.odometry.getPose()
        return nonYPose


    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        #return False
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getChassisSpeed(self) -> ChassisSpeeds:
        #print(f"{self.lastChassisSpeed=}")
        return self.lastChassisSpeed
    


    def updateOdometry(self) -> None:
        
        yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double-90)
        # print(f"{yaw=}")
        a = self.odometry.update(
            yaw,
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            )
        )
        yaw = deg2Rot2d(self.gyro.get_yaw().value_as_double)
        # print(f"{yaw=}")
        a = self.odometry.update(
            yaw,
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            )
        )

        #self.field.setRobotPose(a)

       
    def periodic(self) -> None:
        
        self.updateOdometry()

        if self.shouldUpdateIntakeController:

            self.intake.intakeControllerUpdate()

    def resetMotors(self) -> None:
        pass
                   

     
    def manualDriveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        self.lastChassisSpeed = speeds
        
        speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega)
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

        self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeftOptimized.angle.radians())))
        self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeftOptimized.angle.radians())))
        self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRightOptimized.angle.radians())))
        self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRightOptimized.angle.radians())))

        self.backLeftDrive.set(-backLeftOptimized.speed)
        self.backRightDrive.set(backRightOptimized.speed)
        self.frontLeftDrive.set(frontLeftOptimized.speed)
        self.frontRightDrive.set(frontRightOptimized.speed)


    def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        print(self.getPose())

        self.lastChassisSpeed = speeds

        Vx = speeds.vy
        Vy = speeds.vx
        
        speeds = ChassisSpeeds(-Vx, -Vy, -speeds.omega)
        moduleStates = self.kinematics.toSwerveModuleStates(speeds)
        
        maxModSpeed = 4.1
        frontLeft, frontRight, backLeft, backRight = SwerveDrive4Kinematics.desaturateWheelSpeeds(moduleStates, maxModSpeed)


        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

        self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeftOptimized.angle.radians())))
        self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeftOptimized.angle.radians())))
        self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRightOptimized.angle.radians())))
        self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRightOptimized.angle.radians())))

        #self.backLeftDrive.set(-backLeftOptimized.speed/scale)
        #self.backRightDrive.set(backRightOptimized.speed/scale)
        #self.frontLeftDrive.set(frontLeftOptimized.speed/scale)
        #self.frontRightDrive.set(frontRightOptimized.speed/scale)


        maxVoltage = 13
        self.backLeftDrive.setVoltage(-(backLeftOptimized.speed/maxModSpeed)*maxVoltage)
        self.backRightDrive.setVoltage((backRightOptimized.speed/maxModSpeed)*maxVoltage)
        self.frontLeftDrive.setVoltage((frontLeftOptimized.speed/maxModSpeed)*maxVoltage)
        self.frontRightDrive.setVoltage((frontRightOptimized.speed/maxModSpeed)*maxVoltage)
        

    def stopMotors(self):
        self.frontLeftDrive.set(0)
        self.frontRightDrive.set(0)
        self.backLeftDrive.set(0)
        self.backRightDrive.set(0)

        self.frontLeftRotation.set(0)
        self.frontRightRotation.set(0)
        self.backLeftRotation.set(0)
        self.backRightRotation.set(0)