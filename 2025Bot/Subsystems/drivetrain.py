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

import Subsystems.SwerveModule as SM

kMaxModuleSpeed = 4.72
kMaxRobotSpeed = 4.0

class Drivetrain(commands2.Subsystem):
    def __init__(self):
        #SwerveModule/hardware init
        self.flSM = SM.swerveModule(1, 2, 0, 0.592)
        self.frSM = SM.swerveModule(3, 4, 1, 0.867)
        self.blSM = SM.swerveModule(5, 6, 2, 0.520) 
        self.brSM = SM.swerveModule(7, 8, 3, 0.814)

        self.gyro = phoenix6.hardware.Pigeon2(9)
        self.gyro.set_yaw(0)
        
        #Setting PID and FF constants.
        self.flSM.setDrivePID(0.017352, 0.0, 0.0)
        self.frSM.setDrivePID(0.015337, 0.0, 0.0)
        self.blSM.setDrivePID(0.015337, 0.0, 0.0)
        self.brSM.setDrivePID(0.017195, 0.0, 0.0)

        self.flSM.setRotationPID(0.555, 0.0, 0.002)
        self.frSM.setRotationPID(0.505, 0.0, 0.001)
        self.blSM.setRotationPID(0.51, 0.0, 0.002)
        self.brSM.setRotationPID(0.52, 0.0, 0.00)

        self.flSM.setDriveFF(0.069667, 0.017075, 0.0006183)
        self.frSM.setDriveFF(0.055517, 0.016857, 0.00044453)
        self.blSM.setDriveFF(0.06791, 0.017169, 0.00066838)
        self.brSM.setDriveFF(0.048067, 0.016905, 0.0005649)
        
        #Create a ChassisSpeeds instance.
        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)

        #Location init for kinematics
        frontLeft = Translation2d(.290, .290)
        frontRight = Translation2d(.290, -.290)
        backLeft = Translation2d(-.290, .290)
        backRight = Translation2d(-.290, -.290)

        self.kinematics = SwerveDrive4Kinematics(frontLeft, frontRight, backLeft, backRight)

        #Odometry setup
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                self.flSM.getPosition(),
                self.frSM.getPosition(),
                self.blSM.getPosition(),
                self.brSM.getPosition()
            ),
            Pose2d()
        )

    def reset(self):
        """
        Reset the tracking system.
        """
        self.gyro.set_yaw(0)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                self.flSM.getPosition(),
                self.frSM.getPosition(),
                self.blSM.getPosition(),
                self.brSM.getPosition()
            ),
            Pose2d()
        )
    
    def gyroIsTooHot(self):
        if (self.gyro.get_temperature().value_as_double < 90):
            return False
        else:
            return True
        
    def drivetrainIsTooHot(self):
        if (self.flSM.isTooHot() or self.frSM.isTooHot() or self.blSM.isTooHot() or self.brSM.isTooHot() or self.gyroIsTooHot()):
            return True
        
    def getPose(self):
        """ 
        Returns the current robot pose on the field.
        """
        return self.odometry.getPose()
    
    def resetPose(self, pose2d: wpimath.geometry.Pose2d):
        """
        Reset the tracking system.
        """
        self.gyro.set_yaw(0)

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(),
            (
                self.flSM.getPosition(),
                self.frSM.getPosition(),
                self.blSM.getPosition(),
                self.brSM.getPosition()
            ),
            pose2d
        )
    
    def getChassisSpeeds(self):
        """ 
        Gets the current speed of the robot (x, y, omega).
        """
        return self.chassisSpeeds
    
    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def updateOdometry(self):
        """ 
        Updates the robot angle and the position of all of the wheels.
        """
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
        """ 
        This method stops all of the motors on the drivetrain until the drive command is called again.
        """
        self.flSM.stopAllMotors()
        self.frSM.stopAllMotors()
        self.blSM.stopAllMotors()
        self.brSM.stopAllMotors()

    def drive(self, speeds: ChassisSpeeds) -> None:
        """ 
        Sets each wheel speed in order to make the robot move. 
        """
                
        speeds = ChassisSpeeds(speeds.vx, speeds.vy, speeds.omega)

        # Convert to swerve module states
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds)
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxRobotSpeed #should be 4.6 (MK4I) or 4.72 (Thrifty Bot Swerve) m/s free speed
        )


        self.flSM.setState(swerveModuleStates[0])
        self.frSM.setState(swerveModuleStates[1])
        self.blSM.setState(swerveModuleStates[2])
        self.brSM.setState(swerveModuleStates[3])