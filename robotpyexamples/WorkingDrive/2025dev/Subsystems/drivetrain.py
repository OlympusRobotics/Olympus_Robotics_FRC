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

"""Please Select which version of the Swerve Code that you want to use and the configs related to them"""
#import Subsystems.NewSwerveModule as SM
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

        #Old Swerve Configs
        self.flSM = SM.swerveModule(1, 2, 9, 0.51, 0.0, 0.00) #Tuned
        self.frSM = SM.swerveModule(3, 4, 10, 0.51, 0.0, 0.001) #Tuned
        self.blSM = SM.swerveModule(5, 6, 11, 0.5, 0.0, 0.0) 
        self.brSM = SM.swerveModule(7, 8, 12, 0.51, 0.0, 0.002)

        self.flSM.rotationMotor.setInverted(True)
        self.frSM.rotationMotor.setInverted(True)
        self.blSM.rotationMotor.setInverted(True)
        self.brSM.rotationMotor.setInverted(True)

        self.flSM.driveMotor.setInverted(True)
        self.frSM.driveMotor.setInverted(True)
        self.blSM.driveMotor.setInverted(True)
        self.brSM.driveMotor.setInverted(True)

        


        #New Swerve Configs
        """ self.flSM = SM.swerveModule(1, 2, 0)
        self.frSM = SM.swerveModule(3, 4, 1)
        self.blSM = SM.swerveModule(5, 6, 2)
        self.brSM = SM.swerveModule(7, 8, 3) """

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
                self.flSM.getPosition(),
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


    def driveFO(self, 
        xSpeed: float, 
        ySpeed: float, 
        rotation: float,  
        #periodSeconds: float
    ) -> None:
                
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotation, self.gyro.getRotation2d()
        )

        # Convert to swerve module states
        swerveModuleStates = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, 4.0 #should be 4.6 (MK4I) or 4.72 (Thrifty Bot Swerve) m/s free speed
        )

        self.flSM.setState(swerveModuleStates[0])
        self.frSM.setState(swerveModuleStates[1])
        self.blSM.setState(swerveModuleStates[2])
        self.brSM.setState(swerveModuleStates[3])
        

        
    def driveRO(self, 
        xSpeed: float, 
        ySpeed: float, 
        rotation: float,  
    ) -> None:
        
        """
        Method to drive the robot in Robot Relative mode.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        """
        self.chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rotation, self.gyro.getRotation2d())

        swerveModuleStates = self.kinematics.toSwerveModuleStates(self.chassisSpeeds)
        
        """ wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, 4.0 #should be 4.6 (MK4I) or 4.72 (Thrifty Bot Swerve) m/s free speed
        ) """
        
        self.flSM.setState(swerveModuleStates[0])
        self.frSM.setState(swerveModuleStates[1])
        self.blSM.setState(swerveModuleStates[2])
        self.brSM.setState(swerveModuleStates[3])

    def drive(self, speeds: ChassisSpeeds):
        speeds = ChassisSpeeds(speeds.vx, speeds.vy, speeds.omega)
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

        SwerveModuleState.optimize(frontLeft,
        Rotation2d(ticks2rad(self.flSM.rotationEncoder.get_absolute_position()._value)))

        SwerveModuleState.optimize(frontRight,
        Rotation2d(ticks2rad(self.frSM.rotationEncoder.get_absolute_position()._value)))

        SwerveModuleState.optimize(backLeft,
        Rotation2d(ticks2rad(self.blSM.rotationEncoder.get_absolute_position()._value)))

        SwerveModuleState.optimize(backRight,
        Rotation2d(ticks2rad(self.brSM.rotationEncoder.get_absolute_position()._value)))

        self.flSM.rotationMotor.set(self.flSM.rotationPIDController.calculate(self.flSM.rotationEncoder.get_absolute_position()._value, frontLeft.angle.radians()))
        self.frSM.rotationMotor.set(self.frSM.rotationPIDController.calculate(self.frSM.rotationEncoder.get_absolute_position()._value, frontRight.angle.radians()))
        self.blSM.rotationMotor.set(self.blSM.rotationPIDController.calculate(self.blSM.rotationEncoder.get_absolute_position()._value, frontLeft.angle.radians()))
        self.brSM.rotationMotor.set(self.brSM.rotationPIDController.calculate(self.brSM.rotationEncoder.get_absolute_position()._value, frontRight.angle.radians()))

        self.flSM.driveMotor.set(frontLeft.speed)
        self.frSM.driveMotor.set(frontRight.speed)
        self.blSM.driveMotor.set(backLeft.speed)
        self.brSM.driveMotor.set(backRight.speed)


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