import wpilib
import wpilib.drive
from wpimath import controller
import math
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
import wpilib.drive
from wpilib import DriverStation
from robotcontainer import RobotContainer

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robotContainer = RobotContainer()
        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.command = self.robotContainer.getAutoCommand()
        if self.command:
            self.command.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        """pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        goal_end_vel=0.0, # Goal end velocity in meters/sec
        rotation_delay_distance=0.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )"""



    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

        
        self.drivetrain.gyro.set_yaw(0)        

        
        """self.backLeftRotation.set(-self.BleftPID.calculate(self.BleftEnc.get_absolute_position()._value, 5.0))
        self.frontLeftRotation.set(-self.FleftPID.calculate(self.FleftEnc.get_absolute_position()._value, 5.0))
        self.backRightRotation.set(-self.BrightPID.calculate(self.BrightEnc.get_absolute_position()._value, 5.0))
        self.frontRightRotation.set(-self.FrightPID.calculate(self.FrightEnc.get_absolute_position()._value, 5.0))"""
    
        
 

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        #self.fuckyouhenry.set (1.0)

        self.joystick = wpilib.Joystick(0)

        xspeed = self.joystick.getX()
        yspeed = self.joystick.getY()
        tspeed = self.joystick.getTwist()

        yaw = -self.drivetrain.gyro.get_yaw().value_as_double
        


        h = yaw % 360
        if h < 0:
            h += 360

        h2 = h / 360

        heading = h2 * (math.pi*2)
        
        if abs(xspeed) <.10:
            xspeed=0
        if abs(yspeed) <.10:
            yspeed=0
        if abs(tspeed) <.10:
            tspeed=0


        if False:#xspeed == 0 and yspeed == 0 and tspeed == 0:
            self.backLeftDrive.set(0)
            self.backRightDrive.set(0)
            self.frontLeftDrive.set(0)
            self.frontRightDrive.set(0)

            self.backLeftRotation.set(0)
            self.backRightRotation.set(0)
            self.frontLeftRotation.set(0)
            self.frontRightRotation.set(0)            
        else:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspeed, yspeed, -tspeed, Rotation2d(heading))
            self.drivetrain.driveFromChassisSpeeds(speeds)

    # Convert to module states
            
            """print("HENRY HELP ME")
            print(f"{backLeftOptimized.angle.radians()}, {backLeftOptimized.speed}")
            print(f"{backRightOptimized.angle.radians()}, {backRightOptimized.speed}")
            print(f"{frontLeftOptimized.angle.radians()}, {frontLeftOptimized.speed}")
            print(f"{frontRightOptimized.angle.radians()}, {-frontRightOptimized.speed}")"""

            


if __name__ == "__main__":
    wpilib.run(MyRobot)
