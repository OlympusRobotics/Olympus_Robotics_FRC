
import commands2
import ntcore
import pathplannerlib.auto
import pathplannerlib.config
import wpilib
import wpilib.simulation
import phoenix6
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import Subsystems.drivetrain as drivetrain
import pathplannerlib
from wpimath.kinematics import SwerveModuleState, ChassisSpeeds
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import SmartDashboard

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.drivetrain = drivetrain.Drivetrain()
        #self.config = RobotConfig.fromGUISettings() #Make a pathplanner project in this directory first before uncommenting this line.
        #self.motor = phoenix6.hardware.TalonFX(0)
        #self.orchestra = phoenix6.Orchestra()
        
        # get the default instance of NetworkTables
        nt = ntcore.NetworkTableInstance.getDefault()
        # Start publishing an array of module states with the "/SwerveStates" key
        topic = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState)
        self.pub = topic.publish()

        """ self.orchestra.add_instrument(self.motor)

        self.status = self.orchestra.load_music("ievanpolkka.chrp")

        if not self.status.is_ok():
            print("DONT PLAY IT PLZ") """


        self.timer = wpilib.Timer()
        self.timer.start()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.

    def configureAuto(self):
        AutoBuilder.configure(
            self.drivetrain.odometry.getPose,
            self.drivetrain.reset,
            self.drivetrain.getChassisSpeedsRO,
            lambda speeds, feedforwards: self.drivetrain.driveRO(speeds),
            PPHolonomicDriveController(
                PIDConstants(0.001, 0.0, 0.0),
                PIDConstants(0.001, 0.0, 0.0),
            ),
            self.config,
            self.drivetrain.shouldFlipPath,
            self.drivetrain
            )
        
        


    def robotPeriodic(self): 
        #wpilib.SmartDashboard.putNumber("X Speed", self.xSpeed)
        #wpilib.SmartDashboard.putNumber("Y Speed", self.ySpeed)
        #wpilib.SmartDashboard.putNumber("Rotation", self.rot)
        #SmartDashboard.putData("Field", self.field)
        #self.field.setRobotPose(self.odometry.getPose())
        self.drivetrain.updateOdometry()        
        wpilib.SmartDashboard.putNumber("FLD Temp", self.drivetrain.flSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("FLR Temp", self.drivetrain.flSM.rotationMotor.getMotorTemperature())
        
        wpilib.SmartDashboard.putNumber("FRD Temp", self.drivetrain.frSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("FRR Temp", self.drivetrain.frSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BLD Temp", self.drivetrain.blSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("BLR Temp", self.drivetrain.blSM.rotationMotor.getMotorTemperature())

        wpilib.SmartDashboard.putNumber("BRD Temp", self.drivetrain.brSM.driveMotor.getMotorTemperature())
        wpilib.SmartDashboard.putNumber("BRR Temp", self.drivetrain.brSM.rotationMotor.getMotorTemperature())


        

        return super().robotPeriodic()

    """ def autonomousPeriodic(self) -> None:
        self.drivetrain.updateOdometry() """

    def applyDeadband(self, value, deadband=0.15):
        return value if abs(value) > deadband else 0

    def teleopPeriodic(self) -> None:
        
        """ if (self.controller.getAButton()):
            self.motor.setVoltage(0.5)
            print("A")
        elif (self.controller.getBButton()):
            self.motor.setVoltage(0.5)
            print("B")
        else:
            self.motor.stopMotor()

        if (self.controller.getRightBumperButton()):
            self.orchestra.play()
            print("Bumper")
        elif (self.controller.getLeftBumperButton()):
            self.orchestra.stop()

        if (self.controller.getYButton()):
            print(self.status.is_ok())
            print(self.status.is_error())
            print(self.status.is_warning())
            
        if (self.controller.getXButton()):
            print(self.orchestra.is_playing()) """
        
        
        self.pub.set([self.drivetrain.flSM.getState(),self.drivetrain.frSM.getState(),self.drivetrain.blSM.getState(),self.drivetrain.frSM.getState()])

        #print(str(self.drivetrain.gyro.getRotation2d()))
        
        
        self.xSpeed = self.applyDeadband(self.controller.getLeftY())
        self.ySpeed = self.applyDeadband(self.controller.getLeftX())
        self.rot = self.applyDeadband(self.controller.getRightX())

        print(self.drivetrain.brSM.rotationEncoder.get_absolute_position().value_as_double)

        """ if (self.timer.hasElapsed(1)):
            self.timer.reset()
            print(self.xSpeed)      """   

        if (self.xSpeed == 0 and self.ySpeed == 0 and self.rot == 0):
            self.drivetrain.stopDrivetrain()
        else:
            """ speeds = ChassisSpeeds.fromFieldRelativeSpeeds(-self.xSpeed, -self.ySpeed, -self.rot, self.drivetrain.gyro.getRotation2d())
            self.drivetrain.drive(speeds) """

            self.driveWithJoystick()

            
            
    def driveWithJoystick(self) -> None:
        self.drivetrain.driveFO(-self.xSpeed, -self.ySpeed, -self.rot)