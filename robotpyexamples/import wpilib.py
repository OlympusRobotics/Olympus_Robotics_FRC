import wpilib
import wpilib.drive
import phoenix6 as ctre
import rev
import ntcore
import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

def setPos(motor, speed):
    motor.set(ctre.ControlMode.Position, speed)

def drive(motor, speed):
    motor.set(speed)

def driveCTRE(motor, speed):
    motor.set(ctre.ControlMode.PercentOutput, speed)

class MyRobot(wpilib.TimedRobot):

    def configMotorEncoder(self, motor):
        motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.Analog, 0, 1)
        
        motor.setInverted(False)
        motor.setSensorPhase(True)

        motor.configNominalOutputForward(0)
        motor.configNominalOutputReverse(0)
        motor.configPeakOutputForward(1)
        motor.configPeakOutputReverse(-1)

        motor.config_kF(0, 0.0)
        motor.config_kP(0, 20)
        motor.config_kI(0, 0.0)
        motor.config_kD(0, 2)
        # motor.SetSelectedSensorPosition(0);
        motor.setInverted(True)




    def robotInit(self):
        """This function is called upon program startup and should be used for any initialization code."""
        self.backLeftRotation = ctre.TalonSRX(3)
        self.backRightRotation = ctre.TalonSRX(4)
        self.frontLeftRotation = ctre.TalonSRX(1)
        self.frontRightRotation = ctre.TalonSRX(2)
        
        self.FrontLeftDrive = ctre.TalonSRX(5)
        self.FrontRightDrive = ctre.VictorSPX(6)
        self.BackLeftDrive = ctre.VictorSPX(7)
        self.BackRightDrive = ctre.TalonSRX(8)
        
        
        


        #table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        #self.tx = table.getEntry("tx")
        #self.ty = table.getEntry("ty")
        #self.ta = table.getEntry("ta")
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

        pass
    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""


    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        #self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())
        
        obl = 38
        obr = 550
        ofr = 173
        ofl = 127


        self.configMotorEncoder(self.backLeftRotation)
        self.configMotorEncoder(self.backRightRotation)
        self.configMotorEncoder(self.frontLeftRotation)

        self.frame = 0
            
        self.joystick = wpilib.Joystick(0)

        frontleftlocation = Translation2d(.381, .381) 
        frontrightlocation = Translation2d(.381, -.381) 
        backleftlocation = Translation2d(-.381, .381) 
        backrightlocation = Translation2d(-.381, -.381) 

        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )

# Example chassis speeds: 1 meter per second forward, 3 meters
# per second to the left, and rotation at 1.5 radians per second
# counterclockwise.
        #
        # 
        speeds = ChassisSpeeds(self.joystick.getY(), self.joystick.getX(), self.joystick.getTwist())

# Convert to module states
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

        #table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        #self.tx = table.getEntry("tx")
        #self.ty = table.getEntry("ty")
        #self.ta = table.getEntry("ta")

        self.frame += 1
        mag = self.joystick.getMagnitude()
        rotate = self.joystick.getTwist()
        
        ratio = 1024/(2*math.pi)


        if self.frame % 50 == 0:
            """print(f"Front Left Motor: {self.frontLeftRotation.getSelectedSensorPosition()}")
            print(f"Front Rsdfgdfgight Motor: {self.frontRightRotation.getSelectedSensorPosition()}")
            print(f"Back Left Motor: {self.backLeftRotation.getSelectedSensorPosition()}")
            print(f"Back Right Motor: {self.backRightRotation.getSelectedSensorPosition()}"""
        """speeds = ChassisSpeeds(1.0, 3.0, 1.5)

# Convert to module states
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)"""


        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(self.m_turningEncoder.getDistance()))
        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(self.m_turningEncoder.getDistance()))
        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(self.m_turningEncoder.getDistance()))
        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(self.frontRightRotation.getDistance()))
        
        cock = 1024/2.8444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444

        # The desired field relative speed here is 2 meters per second
        # toward the opponent's alliance station wall, and 2 meters per
        # second toward the left field boundary. The desired rotation
        # is a quarter of a rotation per second counterclockwise. The current
        # robot angle is 45 degrees.
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        2.0, 2.0, math.pi / 2.0, Rotation2d.fromDegrees(180.0))
        # Example module states
        frontLeftState = SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19))
        frontRightState = SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81))
        backLeftState = SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44))
        backRightState = SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56))

        # Convert to chassis speeds
        chassisSpeeds = self.kinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState)

        # Getting individual speeds
        forward = chassisSpeeds.vx
        sideways = chassisSpeeds.vy
        angular = chassisSpeeds.omega

        #speed = chassisSpeeds.vx 
        
#Very Very Very imortant

        # Now use this in our kinematics
        self.moduleStates = self.kinematics.toSwerveModuleStates(speeds)

        setPos(self.backLeftRotation, backLeft.angle.radians()*ratio+obl)
        setPos(self.backRightRotation, backRight.angle.radians()*ratio+obr)
        setPos(self.frontLeftRotation, frontLeft.angle.radians()*ratio+ofl)
        setPos(self.frontRightRotation, frontRight.angle.radians()*ratio+ofr)

        driveCTRE(self.BackLeftDrive, backLeft.speed)
        driveCTRE(self.BackRightDrive, backRight.speed)
        driveCTRE(self.FrontLeftDrive, frontLeft.speed)
        driveCTRE(self.FrontRightDrive, frontRight.speed)

        if self.frame % 50 == 0:
            print(frontLeft.angle.radians())
            print(frontRight.angle.radians())
            print(backLeft.angle.radians())
            print(backRight.angle.radians())
if __name__ == "__main__":
    wpilib.run(MyRobot)

