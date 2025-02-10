import rev
import phoenix6
import math
import wpilib
import commands2
import wpimath.controller
import wpimath.kinematics
import wpimath.trajectory
import wpimath.units
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState

import Subsystems.OldSwerveModule as SM

def ticks2rad(EncoderPositon):
    return EncoderPositon * (2*math.pi)

def gyro2Rot2d(gyro: phoenix6.hardware.Pigeon2):
    yaw = gyro.get_yaw().value_as_double

    h = yaw % 360

    if h < 0:
        h += 360
    
    h2 = h/360

    heading = h2 * (math.pi * 2)

    return Rotation2d(heading)
    
class Drivetrain(commands2.Subsystem):
    def __init__(self):
        #SwerveModule/hardware init
        self.flSM = SM.swerveModule(1, 2, 9, 0.51, 0.0, 0.00)
        self.frSM = SM.swerveModule(3, 4, 10, 0.51, 0.0, 0.001)
        self.blSM = SM.swerveModule(5, 6, 11, 0.5, 0.0, 0.0) 
        self.brSM = SM.swerveModule(7, 8, 12, 0.51, 0.0, 0.002)

        self.gyro = phoenix6.hardware.Pigeon2(13)
        self.gyro.set_yaw(0)

        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)

        #Location init for kinematics
        frontLeft = Translation2d(.324, .324)
        frontRight = Translation2d(.324, -.324)
        backLeft = Translation2d(-.324, .324)
        backRight = Translation2d(-.324, -.324)

        self.kinematics = SwerveDrive4Kinematics(frontLeft, frontRight, backLeft, backRight)

        #Odometry setup
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                self.flSM.getPosition(), #Gets individual swerve module positions (Check Subsystems/OldSwerveModule.py for more details)
                self.frSM.getPosition(),
                self.blSM.getPosition(),
                self.brSM.getPosition()
            ),
            Pose2d()
        )

        super().__init__()

    def reset(self):
        """
        Reset the tracking system.
        """
        self.gyro.set_yaw(0)

        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.flSM.getPosition(),
                self.frSM.getPosition(),
                self.blSM.getPosition(),
                self.brSM.getPosition(),
            ),
            Pose2d()
        )

    def getChassisSpeeds(self):
        return self.chassisSpeeds
    
    def getChassisSpeedsRO(self):
        return self.chassisSpeeds.fromRobotRelativeSpeeds(self.chassisSpeeds, robotAngle=self.gyro.getRotation2d())
    
    def getChassisSpeedsFO(self):
        return self.chassisSpeeds.fromFieldRelativeSpeeds(self.chassisSpeeds, robotAngle=self.gyro.getRotation2d())


    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed


    def driveFO(self, speeds: ChassisSpeeds) -> None:
                
        speeds = ChassisSpeeds(speeds.vx, speeds.vy, speeds.omega)

        # Convert to swerve module states
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds)
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, 4.6 #should be 4.6 (MK4I)
        )

        self.flSM.setState(swerveModuleStates[0])
        self.frSM.setState(swerveModuleStates[1])
        self.blSM.setState(swerveModuleStates[2])
        self.brSM.setState(swerveModuleStates[3])
        

        
    def driveRO(self, speeds: ChassisSpeeds) -> None:        
        """
        Method to drive the robot in Robot Relative mode.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        """

        speeds = ChassisSpeeds(speeds.vx, speeds.vy, speeds.omega)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds)
        
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, 4.6 #should be 4.6m/s free speed (MK4I)
        )
        
        self.flSM.setState(swerveModuleStates[0])
        self.frSM.setState(swerveModuleStates[1])
        self.blSM.setState(swerveModuleStates[2])
        self.brSM.setState(swerveModuleStates[3])


    def updateOdometry(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
            self.flSM.getPosition(),
            self.frSM.getPosition(),
            self.blSM.getPosition(),
            self.brSM.getPosition()
            ),
        )        

    def stopDrivetrain(self):
        self.flSM.stopAllMotors()
        self.frSM.stopAllMotors()
        self.blSM.stopAllMotors()
        self.brSM.stopAllMotors()
