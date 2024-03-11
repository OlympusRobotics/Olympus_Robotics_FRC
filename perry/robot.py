import math
import wpilib
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
import commands2
import drivetrain
import intake
import climber
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
import shooter
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
import Limey
from helper import getInterpAng

class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.drivetrain = drivetrain.DriveTrain()
        self.intake = intake.Intake()
        self.climber = climber.Climber()
        self.shooter = shooter.Shooter()
        self.limey = Limey.Limey()
        
        self.configure_auto()
        #self.globalTimer = time.time()

        self.transferCommand = commands2.SequentialCommandGroup(
            commands2.WaitCommand(.7),
            commands2.InstantCommand(self.stage1, self),
            commands2.WaitCommand(.65),
            commands2.InstantCommand(self.stage2, self),
            commands2.WaitCommand(.5),
            commands2.InstantCommand(self.stage3, self),
            commands2.WaitCommand(.2),
            commands2.InstantCommand(self.stage4, self),
            commands2.WaitCommand(.1),
            commands2.InstantCommand(self.end, self),
        )
    
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


    def getAutonomousCommand(self):
        print("Autocommand Called")
        #self.gyro.set_yaw(0)
        # Load the path you want to follow using its name in the GUI
        #self.shootNamedCommand = self.runOnce(self.shootCommand)
        self.shootCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.PrintCommand("SHOOTER RAN"),
            commands2.WaitCommand(2),
            commands2.PrintCommand("FEED NOTE"),
            commands2.cmd.runOnce( self.shooter.feedNote),
            commands2.WaitCommand(2),
            commands2.PrintCommand("FLYWHEEL COMMAND"),
            commands2.cmd.runOnce(self.shooter.stopFlywheels),
            commands2.cmd.runOnce(lambda: self.shooter.feedMotor.set(0)),
            commands2.PrintCommand("                            SHOOT COMMAND RAN"),
            commands2.cmd.runOnce(lambda: self.intake.intakeDrive.set(1)),
            commands2.WaitCommand(1),
            commands2.cmd.runOnce(self.intake.rotateDown),
        )
        """
        
                commands2.cmd.runOnce(self.shooter.stopFlywheels()),
                commands2.cmd.runOnce(self.shooter.feedMotor.set(0)),
        """

        self.intakeCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.WaitCommand(3),
            commands2.cmd.runOnce(self.intake.rotateHome),
            commands2.WaitCommand(2),
            commands2.PrintCommand("                          INTAKE COMMAND RAN"),
            commands2.cmd.runOnce(self.stage1, self),
            commands2.WaitCommand(.7),
            commands2.cmd.runOnce(self.stage2, self),
            commands2.WaitCommand(1),
            commands2.cmd.runOnce(self.stage3, self),
            commands2.WaitCommand(1),
            commands2.cmd.runOnce(self.stage4, self),
            commands2.WaitCommand(1),
            commands2.cmd.runOnce(self.end, self),
            commands2.PrintCommand("                              TRANS")
            
        )

        #NamedCommands.registerCommand("shoot", commands2.InstantCommand(lambda: self.shootCommand.schedule(), self))
        #NamedCommands.registerCommand("intakeTrans", commands2.InstantCommand(lambda: self.intakeCommand.schedule(), self))
        
        NamedCommands.registerCommand("shoot", self.shootCommand)
        NamedCommands.registerCommand("intakeTrans", self.intakeCommand)
        auto = PathPlannerAuto("testAuto")

        return auto

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.command = self.getAutonomousCommand()
        if self.command:
            self.command.schedule()
        

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    
    def stage1(self):
        self.intake.intakeDrive.set(1)
        self.shooter.feedNote()

    def stage2(self):
        self.intake.intakeDrive.set(-.7)
        print(self.intake.intakeDrive.getOutputCurrent())
        self.shooter.feedNote()
        self.shooter.pushBack()


    def stage3(self):
        self.intake.intakeDrive.set(0)
        self.shooter.feedMotor.set(1)
        self.shooter.pushBack()

    def stage4(self):
        self.shooter.feedMotor.set(0)
        self.shooter.pushBack()

    def end(self):
        self.shooter.stopFlywheels()
        self.shooter.feedMotor.set(0)


    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.drivetrain.resetHarder()   
        self.drivetrain.gyro.set_yaw(0)
        self.transferStartTime = 0

        self.xboxController = wpilib.XboxController(0)
        self.joystick = wpilib.Joystick(2)

    def autoAim(self):
        kP = .03
        tx = self.limey.getLimey()["tx"]
        if tx == 0:
            return -1
        
        arbFF = .04

        return kP * (tx) + arbFF * (tx/abs(tx))


    def shooterAim(self):
        distance = self.limey.getTarget()["distance"]
        angle = self.limey.getTarget()["angle"]

        # if lost tracking, dont change shooter position for smoother operation
        if distance == 0:
            return 0

        # --- if long range ---
        if distance > 2:
            self.shooter.setRot(getInterpAng(distance))

        # --- if close range ---
        # limelight angle is above the horizontal
        angle = 90 - angle

        # angle measured from the top of the shooter to vertical, 0 degrees is up, 90 is horizontal
        # angle to rotations of motor
        rot = 0.210843373494 * angle - 6.45180722892
        self.shooter.setRot(rot-1) #round(rot-1, 1)
        print(rot)

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        
        # ----------------------- DRIVETRAIN CODE -----------------------
        #if not wpilib.DriverStation.getJoystickIsXbox(0):
        
        xspeed = self.joystick.getX()
        yspeed = -self.joystick.getY()

        if not self.joystick.getTrigger():
            tspeed = self.joystick.getTwist()
        else:
            tspeed = self.autoAim()
            if tspeed == -1:
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
            
        if self.xboxController.getXButtonReleased():
            self.transferCommand.schedule()

        if self.xboxController.getXButtonPressed() and self.transferCommand.isScheduled():
            self.transferCommand.cancel()


        # VERY SMART FIX
        if not self.transferCommand.isScheduled():
            intakeButton = self.xboxController.getXButton()     


            self.intake.intakeDrive.set(0)
            self.shooter.stopFlywheels()


            if intakeButton: # if it is currently held
                self.intake.rotateDown()

            else:
                self.intake.rotateHome()

            if self.xboxController.getLeftStickButtonPressed():
                self.climber.rest()
                #self.climber.stopMotors()

            if self.xboxController.getRightStickButtonPressed():
                self.climber.setUp()
                #self.climber.stopMotors()
            
            if self.xboxController.getLeftTriggerAxis() > .5:
                #self.shooter.targetSpeaker()
                self.shooter.spinFlywheels()
                self.shooterAim()


            if self.xboxController.getRightTriggerAxis() > .5:
                self.shooter.feedNote()
            else:
                self.shooter.resetFeed()

            if self.xboxController.getLeftBumper():
                self.shooter.targetAmp()
                self.shooter.spinFlywheels()

            if not self.xboxController.getLeftBumper() and (self.xboxController.getLeftTriggerAxis() < .5) and not self.joystick.getTrigger():
                self.shooter.goHome()

            if self.xboxController.getYButton():
                self.shooter.pushBack()
   

        #self.intake.stopMotors()
        if self.xboxController.getAButton():
            self.intake.intakeDrive.set(1)
        #if xboxController.getBButton():
        #    self.intake.moveDown()
            
        """


        

        
        
        if xboxController.getBButton():
            self.intake.intakeDrive.set(-0.8)

        if xboxController.getAButton():
            self.intake.intakeDrive.set(1)

        """




        
        # auto transfer
        """
          - stage1 - drive intake motor inwards while feed motor intakes
          - stage 2 -eject intake and feed motor
          - 3- spin flywheels backwards
        """




        # ----------------------- SHOOTER CODE -----------------------
"""        revbutton = xboxController.getLeftTriggerAxis()


        if revbutton:
            self.

        """
        #self.drivetrain


if __name__ == "__main__":
    wpilib.run(MyRobot)

