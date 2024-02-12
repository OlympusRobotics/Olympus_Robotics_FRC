import wpilib
import wpilib.drive
from wpimath import controller
import math
from pathplannerlib.auto import AutoBuilder, ReplanningConfig, PathPlannerPath
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
import wpilib.drive
from wpilib import DriverStation
import drivetrain
import commands2
from wpimath import trajectory
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


class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """



        self.drivetrain = drivetrain.DriveTrain()
        self.time = 0.0
        self.configure_auto()

    def autonomousInit(self):
        self.drivetrain.resetHarder()
        
        """This function is run once each time the robot enters autonomous mode."""
        self.drivetrain.gyro.set_yaw(0)
        self.command = self.getAutoCommand()

        if self.command:
            self.command.schedule()
        
        self.drivetrain.resetMotors()
        

        """
        self.time = 0.0
        config = trajectory.TrajectoryConfig(
            0.7,
            1
        )
        
        self.HoloController = controller.HolonomicDriveController(
            controller.PIDController(.1,0,0), controller.PIDController(.1,0,0),
            controller.ProfiledPIDControllerRadians(.5,0,0, trajectory.TrapezoidProfileRadians.Constraints(6.28, 3.14))
        )


        self.myTrajectory = trajectory.TrajectoryGenerator.generateTrajectory(
            Pose2d(0,0,Rotation2d(0)),
            [Translation2d(0,-2), Translation2d(2,0),Translation2d(0,2)],
            Pose2d(0,0,Rotation2d(0)), 
            config
        )
        """

    
    def getAutoCommand(self):
        # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile('test') 

        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)

 

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        """pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        goal_end_vel=0.0, # Goal end velocity in meters/sec
        rotation_delay_distance=0.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )"""

        """
        if self.myTrajectory.totalTime() >= self.time:
            goal = self.myTrajectory.sample(self.time)
            print(goal)
            adjSpeeds = self.HoloController.calculate(
                self.drivetrain.getPose(),
                goal,
                Rotation2d.fromDegrees(0.0)
            )
            self.drivetrain.driveFromChassisSpeeds(adjSpeeds)
            self.time += 0.02
            self.drivetrain.updateOdometry()

        else:
            self.drivetrain.resetMotors()

        """
            
    def configure_auto(self):
        AutoBuilder.configureHolonomic(
            self.drivetrain.getPose,
            self.drivetrain.resetPose,
            self.drivetrain.getChassisSpeed,
            self.drivetrain.driveFromChassisSpeeds,
            HolonomicPathFollowerConfig(
                PIDConstants(.5,0,0),
                PIDConstants(.5,0,0),
                .4,
                .4,
                ReplanningConfig(False)
            ),
            self.drivetrain.shouldFlipPath,
            self.drivetrain
        )

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
        tspeed = -self.joystick.getTwist()

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
            speeds = ChassisSpeeds(xspeed, yspeed, -tspeed)
            self.drivetrain.driveFromChassisSpeeds(speeds)


    # Convert to module states
            
            """print("HENRY HELP ME")
            print(f"{backLeftOptimized.angle.radians()}, {backLeftOptimized.speed}")
            print(f"{backRightOptimized.angle.radians()}, {backRightOptimized.speed}")
            print(f"{frontLeftOptimized.angle.radians()}, {frontLeftOptimized.speed}")
            print(f"{frontRightOptimized.angle.radians()}, {-frontRightOptimized.speed}")"""

            


if __name__ == "__main__":
    wpilib.run(MyRobot)
