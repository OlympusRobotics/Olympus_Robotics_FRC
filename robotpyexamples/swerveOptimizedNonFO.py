import wpilib
import wpilib.drive
import rev
import phoenix6 as ctre
import wpimath 
from wpimath import controller
import ntcore
import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import wpilib.drive

# convert radians to encoder range
def lratio(angle):
    if angle > math.pi:
        return float(((math.pi-angle)/math.pi)*.5)
    elif angle < math.pi:
        return float(((angle)/math.pi)*-.5)

    return 0

# converts encoder output to radians
def ticks2rad(something):
    if something <= 0 and something >= -.5:
        return something-(2*math.pi)
    else:
        return 2* (.5-something) * math.pi + math.pi


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called pon program startup and
        should be used for any initialization code.
        """
        self.backLeftRotation = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightRotation = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)

        self.backLeftDrive = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftDrive = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        
        self.BleftEnc = ctre.hardware.CANcoder(11)
        self.BrightEnc = ctre.hardware.CANcoder(13)
        self.FleftEnc = ctre.hardware.CANcoder(10)
        self.FrightEnc = ctre.hardware.CANcoder(12)
        

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""


    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
    


    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

       
        self.BleftPID = controller.PIDController(4.2,0,.0001)
        self.BleftPID.enableContinuousInput(-.5,.5)
        self.BleftPID.setSetpoint(0.0)
        self.BrightPID = controller.PIDController(4.2,0,.0001)
        self.BrightPID.enableContinuousInput(-.5,.5)
        self.BrightPID.setSetpoint(0.0)
        self.FleftPID = controller.PIDController(4.2,0,.0001)
        self.FleftPID.enableContinuousInput(-.5,.5)
        self.FleftPID.setSetpoint(0.0)
        self.FrightPID = controller.PIDController(4.2,0,.0001)
        self.FrightPID.enableContinuousInput(-.5,.5)
        self.FrightPID.setSetpoint(0.0)

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

        self.joystick = wpilib.Joystick(0)

        frontrightlocation = Translation2d(.381, .381) 
        frontleftlocation = Translation2d(.381, -.381) 
        backleftlocation = Translation2d(-.381, -.381)         
        backrightlocation = Translation2d(-.381, .381)         

        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )


        speeds = ChassisSpeeds(self.joystick.getY(), -self.joystick.getX(), self.joystick.getTwist())

        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(speeds)

        #table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
        #self.tx = table.getEntry("tx")
        #self.ty = table.getEntry("ty")
        #self.ta = table.getEntry("ta")

            
        # optimization
        frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        Rotation2d(ticks2rad(self.FleftEnc.get_absolute_position()._value)))
        frontRightOptimized = SwerveModuleState.optimize(frontRight,
        Rotation2d(ticks2rad(self.FrightEnc.get_absolute_position()._value)))
        backLeftOptimized = SwerveModuleState.optimize(backLeft,
        Rotation2d(ticks2rad(self.BleftEnc.get_absolute_position()._value)))
        backRightOptimized = SwerveModuleState.optimize(backRight,
        Rotation2d(ticks2rad(self.BrightEnc.get_absolute_position()._value)))

        # use pidcontroller to move swerve wheel to calculated angle
        self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, lratio(backLeftOptimized.angle.radians())))
        self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, lratio(frontLeftOptimized.angle.radians())))
        self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, lratio(backRightOptimized.angle.radians())))
        self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, lratio(frontRightOptimized.angle.radians())))

        # set speeds
        self.backLeftDrive.set(backLeftOptimized.speed)
        self.backRightDrive.set(backRightOptimized.speed)
        self.frontLeftDrive.set(frontLeftOptimized.speed)
        self.frontRightDrive.set(-frontRightOptimized.speed)
        


if __name__ == "__main__":
    wpilib.run(MyRobot)