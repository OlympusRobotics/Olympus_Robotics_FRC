import wpilib
import wpilib.drive
import phoenix5 as ctre
import math
import wpimath
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters 

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.frontRightDrive = ctre.WPI_TalonSRX(0)
        self.backRightDrive = ctre.WPI_TalonSRX(1)
        self.rightGroup = wpilib.MotorControllerGroup(self.frontRightDrive, self.backRightDrive)

        self.backLeftDrive = ctre.WPI_TalonSRX(2)
        self.frontLeftDrive = ctre.WPI_TalonSRX(3)
        self.leftGroup = wpilib.MotorControllerGroup(self.frontLeftDrive, self.backLeftDrive)
        
        #self.frontRightDrive.setInverted(True)
        #self.backRightDrive.setInverted(True)
        
        self.drive = wpilib.drive.DifferentialDrive(self.leftGroup, self.rightGroup)
        #self.drive = wpilib.drive.DifferentialDrive(self.frontLeftDrive, self.frontRightDrive)
        #self.drive2 = wpilib.drive.DifferentialDrive(self.backLeftDrive, self.backLeftDrive)

        #self.joystick = wpilib.Joystick(0)
        self.controller = wpilib.XboxController(0)
        
        
    #Tank drive movement commands (Wpilib differential drive does not work with TalonSRX)
        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""


    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        

    def teleopPeriodic(self):


        kinematics = DifferentialDriveKinematics(inchesToMeters(27.0))

        chassisSpeeds = ChassisSpeeds(self.controller.getRightX(), 0, self.controller.getLeftY()*2.5)

        wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)

        leftVelocity = wheelSpeeds.left

        rightVelocity = wheelSpeeds.right

        self.frontLeftDrive.set(leftVelocity)
        self.frontRightDrive.set(rightVelocity)
        self.backLeftDrive.set(leftVelocity)
        self.backRightDrive.set(rightVelocity)

        

        #print(f"left vel: {leftVelocity}")
        #print(f"right vel: {rightVelocity}")
        
        
            
if __name__ == "__main__":
    wpilib.run(MyRobot)