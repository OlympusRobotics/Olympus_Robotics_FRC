import random
import math
import commands2
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from pathplannerlib.geometry_util import flipFieldPos, flipFieldRotation, flipFieldPose
from wpimath import controller
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
import phoenix6 as ctre
from wpilib import DriverStation
from wpilib import SmartDashboard, Field2d
import ntcore
import wpilib
import wpilib.drive
import phoenix5 as ctre
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

def setPos(motor, speed):
    motor.set(ctre.ControlMode.Position, speed)

def drive(motor, speed):
    motor.set(speed)

def driveCTRE(motor, speed):
    motor.set(ctre.ControlMode.PercentOutput, speed)

def lratio(angle):
    """converts -pi, pi to -.5,.5"""
    return ((angle/math.pi)*-.5)

def ticks2rad(something):
    return (something/.5)*-math.pi

def FUCKticks2rad(something):

    return (something/1024) * 2* math.pi

def deg2Rot2d(deg) -> Rotation2d:
    yaw = -deg.value_as_double/360
    return Rotation2d(yaw * math.pi * 2)
    """
    yaw = -deg.value_as_double

    if yaw < 0:
        yaw += 360

    yaw = yaw/360


    return Rotation2d(yaw * (math.pi*2))"""


def getSwerveModPos(rotEnc : ctre.TalonSRX, driveEnc: wpilib.Encoder) -> SwerveModulePosition:

    return SwerveModulePosition(
        driveEnc.getDistance(),
        Rotation2d(FUCKticks2rad(rotEnc.getSelectedSensorPosition()))
    )


class DriveTrain(commands2.Subsystem):
    def configMotorEncoder(self, motor):
        motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.Analog, 0, 1)
        
        motor.setInverted(False)
        motor.setSensorPhase(True)

        motor.configNominalOutputForward(0)
        motor.configNominalOutputReverse(0)
        motor.configPeakOutputForward(1)
        motor.configPeakOutputReverse(-1)

        motor.config_kF(0, 0.0)
        motor.config_kP(0, 10)
        motor.config_kI(0, 0.0)
        motor.config_kD(0, 2)
        # motor.SetSelectedSensorPosition(0);
        motor.setInverted(True)
        


    def __init__(self) -> None:
        super().__init__()
        #self.backLeftRotation1 = rev.CANSparkMax(71, rev.CANSparkMax.MotorType.kBrushless)
        print("INIT CALLED INIT")
        
        
        self.backLeftRotation = ctre.TalonSRX(3)
        self.backRightRotation = ctre.TalonSRX(4)
        self.frontLeftRotation = ctre.TalonSRX(1)
        self.frontRightRotation = ctre.TalonSRX(2)
        
        self.FrontLeftDrive = ctre.TalonSRX(5)
        self.FrontRightDrive = ctre.VictorSPX(6)
        self.BackLeftDrive = ctre.VictorSPX(7)
        self.BackRightDrive = ctre.TalonSRX(8)

        




        
        self.frontLeftDriveEncoder = wpilib.Encoder(2,3)
        self.frontRightDriveEncoder = wpilib.Encoder(8,9)
        self.backLeftDriveEncoder = wpilib.Encoder(0,1)
        self.backRightDriveEncoder = wpilib.Encoder(6,7)

        dpp = ((2*math.pi*.09)/20.0)/6.67
        self.frontLeftDriveEncoder.setDistancePerPulse(dpp)
        self.frontRightDriveEncoder.setDistancePerPulse(dpp)
        self.backLeftDriveEncoder.setDistancePerPulse(dpp)
        self.backRightDriveEncoder.setDistancePerPulse(dpp)

        
        

        self.obl = 125
        self.obr = 536
        self.ofr = 124
        self.ofl = 346

        self.configMotorEncoder(self.backLeftRotation)
        self.configMotorEncoder(self.backRightRotation)
        self.configMotorEncoder(self.frontLeftRotation)
        self.configMotorEncoder(self.frontRightRotation)

        self.gyro = phoenix6.hardware.Pigeon2(14)
        self.gyro.set_yaw(0)

        frontrightlocation = Translation2d(.381, -.381) 
        frontleftlocation = Translation2d(.381, .381) 
        backleftlocation = Translation2d(-.381, .381)         
        backrightlocation = Translation2d(-.381, -.381)         

        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )
        self.lastChassisSpeed = ChassisSpeeds(0,0,0)


        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                getSwerveModPos(self.frontLeftRotation, self.frontLeftDriveEncoder),
                getSwerveModPos(self.frontRightRotation, self.frontRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder)

            ),
            Pose2d()

        )


        
        




    def shootCommand(self):
        return print("FUCKYOU")
        

    def getAutonomousCommand(self):
        print("Autocommand Called")
        self.gyro.set_yaw(0)
        # Load the path you want to follow using its name in the GUI
        self.shootNamedCommand = self.runOnce(self.shootCommand)
        NamedCommands.registerCommand("shoot", self.shootNamedCommand)
        auto = PathPlannerAuto("testAuto")

        return auto


    def resetHarder(self, initialPose = Pose2d()):
        print("CALLED RSETHARDER")
                
        """        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                    initialPose = flipFieldPose(initialPose)
        """
        self.gyro.set_yaw(initialPose.rotation().degrees())

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            deg2Rot2d(self.gyro.get_yaw()),
            (
                getSwerveModPos(self.frontLeftRotation, self.frontLeftDriveEncoder),
                getSwerveModPos(self.frontRightRotation, self.frontRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder)
            ),
            Pose2d(-initialPose.X(),-initialPose.Y(), initialPose.rotation())

        )

        print(self.getPose())


        

    def resetMotors(self):
        driveCTRE(self.BackLeftDrive, 0)
        driveCTRE(self.BackRightDrive, 0)
        driveCTRE(self.FrontLeftDrive, 0)
        driveCTRE(self.FrontRightDrive, 0)
        setPos(self.backLeftRotation, self.obl)
        setPos(self.backRightRotation, self.obr)
        setPos(self.frontLeftRotation, self.ofl)
        setPos(self.frontRightRotation, self.ofr)
        

    def getPose(self):
        nonYPose = self.odometry.getPose()
        return(Pose2d(-nonYPose.X(), -nonYPose.Y(), nonYPose.rotation()))



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
        
        yaw = deg2Rot2d(self.gyro.get_yaw())
        # print(f"{yaw=}")
        a = self.odometry.update(
            yaw,
            (
                getSwerveModPos(self.frontLeftRotation, self.frontLeftDriveEncoder),
                getSwerveModPos(self.frontRightRotation, self.frontRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder),
                getSwerveModPos(self.backRightRotation, self.backRightDriveEncoder)
            )
        )

        #self.field.setRobotPose(a)

       
    def periodic(self) -> None:
        self.updateOdometry()
        #.publisher.set(self.kinematics.toSwerveModuleStates(self.lastChassisSpeed))
        # print(f"periodic odometryu FUCK: {self.odometry.getPose()}")

    def testGetPose(self) -> Pose2d:
        return Pose2d()
                   

       
    def driveFromChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        if random.random() < .2:
            print(self.getPose())
            print(f"{speeds}\n")
            
        Vx = -speeds.vy
        Vy = speeds.vx
        Omega = speeds.omega
        self.lastChassisSpeed = speeds
        speeds = ChassisSpeeds(Vx, Vy, Omega)#Omega/4
        
        #speeds = ChassisSpeeds(speeds.vy, speeds.vx, -speeds.omega)

        
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)


        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(FUCKticks2rad(self.frontLeftRotation.getSelectedSensorPosition()-self.ofl)))

        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(FUCKticks2rad(self.frontRightRotation.getSelectedSensorPosition())))

        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(FUCKticks2rad(self.backLeftRotation.getSelectedSensorPosition())))

        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(FUCKticks2rad(self.backRightRotation.getSelectedSensorPosition()+self.obr)))

        ratio = 1024/(2*math.pi)

        #print(self.frontLeftRotation.getSelectedSensorPosition())
        #print(backLeftOptimized.angle.radians())
        #print(frontRightOptimized.angle.radians())

    

        setPos(self.backLeftRotation, backLeftOptimized.angle.radians()*ratio+self.obl)
        setPos(self.backRightRotation, backRightOptimized.angle.radians()*ratio+self.obr)
        setPos(self.frontLeftRotation, frontLeftOptimized.angle.radians()*ratio+self.ofl)
        setPos(self.frontRightRotation, frontRightOptimized.angle.radians()*ratio+self.ofr)

        driveCTRE(self.BackLeftDrive, backLeftOptimized.speed)
        driveCTRE(self.BackRightDrive, backRightOptimized.speed)
        driveCTRE(self.FrontLeftDrive, frontLeftOptimized.speed)
        driveCTRE(self.FrontRightDrive, frontRightOptimized.speed)




            #print(lratio(backRightOptimized.angle.radians()))
            #print(self.BrightEnc.get_absolute_position()._value)

    def manualDrive(self, speeds: ChassisSpeeds) -> None:
        if random.random() < .2:
            print(self.getPose())
            print(f"{speeds}\n")
            
    
        speeds = ChassisSpeeds(-speeds.vy, speeds.vx, -speeds.omega)

        
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)


        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(FUCKticks2rad(self.frontLeftRotation.getSelectedSensorPosition()-self.ofl)))

        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(FUCKticks2rad(self.frontRightRotation.getSelectedSensorPosition())))

        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(FUCKticks2rad(self.backLeftRotation.getSelectedSensorPosition())))

        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(FUCKticks2rad(self.backRightRotation.getSelectedSensorPosition()+self.obr)))

        ratio = 1024/(2*math.pi)

        #print(self.frontLeftRotation.getSelectedSensorPosition())
        #print(backLeftOptimized.angle.radians())
        #print(frontRightOptimized.angle.radians())

    

        setPos(self.backLeftRotation, backLeftOptimized.angle.radians()*ratio+self.obl)
        setPos(self.backRightRotation, backRightOptimized.angle.radians()*ratio+self.obr)
        setPos(self.frontLeftRotation, frontLeftOptimized.angle.radians()*ratio+self.ofl)
        setPos(self.frontRightRotation, frontRightOptimized.angle.radians()*ratio+self.ofr)

        driveCTRE(self.BackLeftDrive, backLeftOptimized.speed)
        driveCTRE(self.BackRightDrive, backRightOptimized.speed)
        driveCTRE(self.FrontLeftDrive, frontLeftOptimized.speed)
        driveCTRE(self.FrontRightDrive, frontRightOptimized.speed)




            #print(lratio(backRightOptimized.angle.radians()))
            #print(self.BrightEnc.get_absolute_position()._value)
