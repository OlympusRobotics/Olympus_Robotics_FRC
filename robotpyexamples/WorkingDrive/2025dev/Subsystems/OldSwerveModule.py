import rev
import phoenix6
import math
import wpilib
import commands2
import wpimath.controller
import wpimath.kinematics
import wpimath.units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


kWheelRadius = 0.0508 #Wheel radius in Meters
kDriveEncoderRes = 4096 #SparkMAX Enocder Resolution
kMaxAngularVelocity = math.pi
kMaxAngularAcceleration = math.tau
kGearRatio = 6.75

def NEOtoDistance(EncoderPosition) -> float: #Converts the current position of the Motor (rotations) into a unit of distance traveled (Meters)
    return (EncoderPosition * math.pi * (2*kWheelRadius) / (kDriveEncoderRes * kGearRatio))

def rpm2mps(rotations) -> float: #Converts from rotations per minute to meters per Second
    rps = rotations / 60.0
    rpsWithRatio = rps / kGearRatio
    speed = rpsWithRatio * (2 * math.pi * kWheelRadius)
    return speed

def ticks2rad(EncoderPositon):
    return EncoderPositon * (2*math.pi)

class swerveModule(commands2.Subsystem):
    def __init__(
        self,
        DriveMotorID: int,
        RotationMotorID: int,
        RotationEncoderID: int,
        kP: float,
        kI: float,
        kD: float
        ) -> None:
        
        """
        Initalizes an instance of a swerve module.       
        """
        
        #Hardware init
        self.driveMotor = rev.SparkMax(DriveMotorID, rev.SparkMax.MotorType.kBrushless)
        self.rotationMotor = rev.SparkMax(RotationMotorID, rev.SparkMax.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.rotationEncoder = phoenix6.hardware.CANcoder(RotationEncoderID)

    
        #PID Setup
        self.drivePIDController = wpimath.controller.PIDController(
            0,  # Proportional gain
            0.00,   # Integral gain
            0.0,   # Derivative gain
        )
        
        self.rotationPIDController = wpimath.controller.PIDController(
            kP,  # Proportional gain
            kI,   # Integral gain
            kD,   # Derivative gain
        )

        self.rotationPIDController.enableContinuousInput(-math.pi, math.pi)
        self.rotationPIDController.setSetpoint(0.0)
        
        #Feed Forward Control
        self.driveMotorFeedForward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.rotationMotorFeedForward = wpimath.controller.SimpleMotorFeedforwardRadians(1, 0.2)


        super().__init__()
        
    def getState(self):
        """
        Gets the current state of a single swerve module (Both Drive and Rotation motors)
        """
        return SwerveModuleState(
            rpm2mps(self.driveEncoder.getVelocity()), #Gets the speed of the wheels in m/s
            Rotation2d(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double)) #Converts the position into radians as rotation2d requests
        )        
    
    def getPosition(self):
        """
        Gets the current position of a swerve module (both the drive and rotation motors)
        """
        return SwerveModulePosition(
            NEOtoDistance(self.driveEncoder.getPosition()), #gets the current position of the wheels
            Rotation2d(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double)) #Converts the position into radians as rotation2d requests
        )
    
    def setState(
            self,
            newState: SwerveModuleState
    ) -> None:
        """
        Sets a new state for the swerve module to move to.
        """
        #SwerveModuleState.optimize(newState, Rotation2d(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double)))

        newState.optimize(Rotation2d(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double)))
        newState.cosineScale(Rotation2d(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double)))

        driveOutput = self.drivePIDController.calculate(self.driveMotor.get(), newState.speed)
        #driveFF = self.driveMotorFeedForward.calculate(newState.speed)

        rotationOutput = self.rotationPIDController.calculate(ticks2rad(self.rotationEncoder.get_absolute_position().value_as_double), newState.angle.radians())
        rotationFF = self.rotationMotorFeedForward.calculate(self.rotationPIDController.getSetpoint())


        #self.driveMotor.set_control(phoenix6.controls.VoltageOut(driveOutput + driveFF))
        #self.rotationMotor.setVoltage(rotationOutput + rotationFF)

        #self.driveMotor.set(driveOutput)
        self.driveMotor.set(newState.speed)
        self.rotationMotor.set(rotationOutput)
        #self.rotationMotor.set(rotationOutput)

    def stopAllMotors(self):
        self.driveMotor.stopMotor()
        self.rotationMotor.stopMotor()