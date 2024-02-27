import math
import wpilib
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
import commands2
import drivetrain
import intake
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
import shooter

class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.drivetrain = drivetrain.DriveTrain()
        self.intake = intake.Intake()
        self.shooter = shooter.Shooter()

        self.configure_auto()
    
    def configure_auto(self):
        AutoBuilder.configureHolonomic(
            self.drivetrain.getPose,
            self.drivetrain.resetHarder,
            self.drivetrain.getChassisSpeed,
            self.drivetrain.driveFromChassisSpeeds,
            HolonomicPathFollowerConfig(
                PIDConstants(0,0,0),
                PIDConstants(3,0,0),
                4.602,
                .36,
                ReplanningConfig(False)
            ),
            self.drivetrain.shouldFlipPath,
            self.drivetrain
        )

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.command = self.drivetrain.getAutonomousCommand()
        if self.command:
            self.command.schedule()
        

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass



    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.drivetrain.resetHarder()   
        self.drivetrain.gyro.set_yaw(0)
 

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        
        # ----------------------- DRIVETRAIN CODE -----------------------
        #if not wpilib.DriverStation.getJoystickIsXbox(0):
        self.joystick = wpilib.Joystick(2)
        xspeed = self.joystick.getX()
        yspeed = -self.joystick.getY()
        tspeed = self.joystick.getTwist()
            
        #else:
        #    self.joystick = wpilib.XboxController(0)
        #    xspeed = self.joystick.getLeftX()
        #    yspeed = self.joystick.getLeftY()
        #    tspeed = self.joystick.getRightX()

        #self.joystick = wpilib.XboxController(1)
        #xspeed = self.joystick.getLeftX()/2
        #yspeed = -self.joystick.getLeftY()/2
        #tspeed = self.joystick.getRightX()/2
        yaw = self.drivetrain.gyro.get_yaw().value_as_double

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
            self.drivetrain.manualDriveFromChassisSpeeds(speeds)


        # ----------------------- INTAKE CODE -----------------------
        #self.intake.rotateDown()
        xboxController = wpilib.XboxController(1)
        intakeButton = xboxController.getXButton()
        if intakeButton: # if it is currently held
            self.intake.rotateDown()

        else:
            self.intake.rotateHome()

        self.intake.stopMotors()
        if xboxController.getAButton():
            self.intake.moveUp()
        if xboxController.getBButton():
            self.intake.moveDown()



        # ----------------------- PASS NOTE FROM INTAKE TO SHOOTER -----------------------
        passButton = xboxController.getAButton()
        if passButton:
            self.intake.expel() # slowly roll then note into the shooter, hopefully the shooter will have enough grip to stall the motor
            if self.shooter.grabNote(): # get note into ready position with encoders
                self.intake.stopIntake() # stop intake motor
        else:
            self.shooter.resetFeed() # reset enc pos
        

        # ----------------------- SHOOTER CODE -----------------------
        shooterButton = xboxController.getRightTriggerAxis()
        if shooterButton > 0: # if trigger pressed, spin wheels and when max vel reached, feed note
            if self.shooter.spinFlywheels():
                self.shooter.feedNote()

        else:
            # stop motors
            self.shooter.resetFeed()
            self.shooter.stopFlywheels()
        
        #self.drivetrain


if __name__ == "__main__":
    wpilib.run(MyRobot)



"""


        self.backLeftRotation = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightRotation = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftRotation = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightRotation = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)

        self.backLeftDrive = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
        self.backRightDrive = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.frontLeftDrive = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.frontRightDrive = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        

        self.frontRightDriveEnc = self.frontRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.frontLeftDriveEnc = self.frontLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.backRightDriveEnc = self.backRightDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.backLeftDriveEnc = self.backLeftDrive.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)


        self.BleftEnc = ctre.hardware.CANcoder(11)
        self.BrightEnc = ctre.hardware.CANcoder(13)
        self.FleftEnc = ctre.hardware.CANcoder(10)
        self.FrightEnc = ctre.hardware.CANcoder(12)

        self.lastChassisSpeed = ChassisSpeeds(0, 0, 0)

        Kp = 1.5
        self.BleftPID = controller.PIDController(Kp,0,.000)
        self.BleftPID.enableContinuousInput(-.5,.5)
        self.BleftPID.setSetpoint(0.0)
        self.BrightPID = controller.PIDController(3.5,0,.000)
        self.BrightPID.enableContinuousInput(-.5,.5)
        self.BrightPID.setSetpoint(0.0)
        self.FleftPID = controller.PIDController(Kp,0,.000)
        self.FleftPID.enableContinuousInput(-.5,.5)
        self.FleftPID.setSetpoint(0.0)
        self.FrightPID = controller.PIDController(Kp,0,.000)
        self.FrightPID.enableContinuousInput(-.5,.5)
        self.FrightPID.setSetpoint(0.0)



        self.gyro = ctre.hardware.Pigeon2(14)

        
        frontrightlocation = Translation2d(.381, .381) 
        frontleftlocation = Translation2d(.381, -.381) 
        backleftlocation = Translation2d(-.381, -.381)         
        backrightlocation = Translation2d(-.381, .381)       


        self.kinematics = SwerveDrive4Kinematics(
            frontleftlocation, frontrightlocation, backleftlocation, backrightlocation
        )



       

       
"""