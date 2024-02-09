import random
import rev
import math
import commands2
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpimath import controller
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import phoenix6 as ctre
from wpilib import DriverStation
from wpilib import SmartDashboard, Field2d
import ntcore

def lratio(angle):
    """converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    return (something/.5)*-math.pi

def FUCKticks2rad(something):
    return something * 2* math.pi

def deg2Rot2d(deg) -> Rotation2d:
    yaw = deg.value_as_double/360
    return Rotation2d(yaw * math.pi * 2)
    """
    yaw = -deg.value_as_double

    if yaw < 0:
        yaw += 360

    yaw = yaw/360


    return Rotation2d(yaw * (math.pi*2))"""

def deg2Rot2da(deg) -> Rotation2d:
    yaw = -deg.value_as_double
    
    if yaw < 0:
        h = 360-(yaw % 360)
    else:
        h = yaw % 360

    h2 = h / 360

    return Rotation2d(h2 * (math.pi*2))

def getSwerveModPos(rotEnc : ctre.hardware.CANcoder, driveEnc: rev.SparkRelativeEncoder) -> SwerveModulePosition:

    return SwerveModulePosition(
        driveEnc.getPosition()*.3191858,
        Rotation2d(FUCKticks2rad(rotEnc.get_position().value_as_double))
    )


class DriveTrain(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        #self.field = Field2d()


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


        self.BleftEnc = ctre.hardware.CANcoder(11)
        self.BrightEnc = ctre.hardware.CANcoder(13)
        self.FleftEnc = ctre.hardware.CANcoder(10)
        self.FrightEnc = ctre.hardware.CANcoder(12)

        self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

        Kp = 1.5
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



        self.gyro = ctre.hardware.Pigeon2(14)
        self.gyro.set_yaw(0)

        
        frontrightlocation = Translation2d(.381, .381) 
        frontleftlocation = Translation2d(.381, -.381) 
        backleftlocation = Translation2d(-.381, -.381)         
        backrightlocation = Translation2d(-.381, .381)       
        """  
        frontleftlocation = Translation2d(.381, .381) 
        frontrightlocation = Translation2d(.381, -.381) 
        backrightlocation = Translation2d(-.381, -.381)         
        backleftlocation = Translation2d(-.381, .381)         
        """


        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            deg2Rot2d(self.gyro.get_yaw()),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            Pose2d(0,0, Rotation2d(0))
        )
        

        #SmartDashboard.putData("penis", self.field)
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("testTable")
    
        self.publisher = ntcore.NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState).publish()


        

    def getAutonomousCommand(self):
        self.gyro.set_yaw(0)
        # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile('test') 

        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)
    
    def resetPose(self, pose):
        print("RESET POSE")
        self.odometry.resetPosition(
            Rotation2d(0),
            (
                getSwerveModPos(self.FleftEnc, self.frontLeftDriveEnc),
                getSwerveModPos(self.FrightEnc, self.frontRightDriveEnc),
                getSwerveModPos(self.BleftEnc, self.backLeftDriveEnc),
                getSwerveModPos(self.BrightEnc, self.backRightDriveEnc)
            ),
            pose
        )

    def getPose(self):
        print("FUCKFUCKFUCKFUCKFUCKFUCK")
        return(self.odometry.getPose())



    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getChassisSpeed(self) -> ChassisSpeeds:
        #print(f"{self.lastChassisSpeed=}")
        return self.lastChassisSpeed
    


    def updateOdometry(self) -> None:
        
        yaw = deg2Rot2d(self.gyro.get_yaw())
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
        self.publisher.set(self.kinematics.toSwerveModuleStates(self.lastChassisSpeed))
        # print(f"periodic odometryu FUCK: {self.odometry.getPose()}")

    def testGetPose(self) -> Pose2d:
        return Pose2d()
                   
    def resetMotors(self) -> None:
 
        self.backLeftRotation.set(0)
        self.frontLeftRotation.set(0)
        self.backRightRotation.set(0)
        self.frontRightRotation.set(0)

        self.backLeftDrive.set(0)
        self.backRightDrive.set(0)
        self.frontLeftDrive.set(0)
        self.frontRightDrive.set(0)

       
    def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:

        
        
        #speeds = ChassisSpeeds(speeds.vx, -speeds.vy, -speeds.omega) #-speeds.omega
        test = -speeds.vy
        #speeds = ChassisSpeeds(speeds.vx, test, speeds.omega) #-speeds.omega
        self.lastChassisSpeed = speeds
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

        self.backLeftDrive.set(backLeftOptimized.speed)
        self.backRightDrive.set(backRightOptimized.speed)
        self.frontLeftDrive.set(frontLeftOptimized.speed)
        self.frontRightDrive.set(-frontRightOptimized.speed)

        

        if True:
            print(self.odometry.getPose())
            print(f"{speeds}\n")
            print(f"{test}")

            #print(lratio(backRightOptimized.angle.radians()))
            #print(self.BrightEnc.get_absolute_position()._value)

