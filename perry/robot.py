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
from wpilib import DriverStation

class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.drivetrain = drivetrain.DriveTrain()
        self.climber = climber.Climber()
        self.shooter = shooter.Shooter()
        self.limey = Limey.Limey()
        
        self.configure_auto()
        self.globalTimer = wpilib.Timer()
        self.globalTimer.start()

        import time
        self.transferCommand = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.drivetrain.intake.rotateHome, self),
            #commands2.InstantCommand(self.shooter.goHome, self),
            commands2.InstantCommand(self.drivetrain.intake.transferHome, self),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(.8), self),
            commands2.WaitCommand(1.3),
            commands2.InstantCommand(self.stage1, self),
            #commands2.WaitCommand(.75),
            commands2.WaitCommand(1.25),
            commands2.InstantCommand(self.stage2, self),
            commands2.WaitCommand(.5),
            commands2.InstantCommand(self.stage3, self),
            commands2.WaitCommand(.2),
            commands2.InstantCommand(self.stage4, self),
            commands2.WaitCommand(.1),
            commands2.InstantCommand(self.end, self),
        )

        self.shooterot = 0
        self.shooterInte = 0


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
            commands2.cmd.runOnce(lambda: self.drivetrain.driveFromChassisSpeeds(ChassisSpeeds(0,0,0))),
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.PrintCommand("SHOOTER RAN"),
            commands2.WaitCommand(1),
            commands2.PrintCommand("FEED NOTE"),
            commands2.cmd.runOnce( self.shooter.feedNote),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(-1)),
            commands2.WaitCommand(1),
            commands2.PrintCommand("FLYWHEEL COMMAND"),
            commands2.cmd.runOnce(self.shooter.stopFlywheels),
            commands2.cmd.runOnce(lambda: self.shooter.feedMotor.set(0)),
            commands2.PrintCommand("                            SHOOT COMMAND RAN"),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(1)),
            commands2.cmd.runOnce(self.drivetrain.intake.rotateDown),
            commands2.WaitCommand(.5)
        )
        """
        
                commands2.cmd.runOnce(self.shooter.stopFlywheels()),
                commands2.cmd.runOnce(self.shooter.feedMotor.set(0)),
        """

        self.drivetrain.intakeCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(self.drivetrain.intake.rotateHome),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(1)),
            commands2.WaitCommand(1.3),
        )

        #NamedCommands.registerCommand("shoot", commands2.InstantCommand(lambda: self.shootCommand.schedule(), self))
        #NamedCommands.registerCommand("intakeTrans", commands2.InstantCommand(lambda: self.drivetrain.intakeCommand.schedule(), self))
        
        NamedCommands.registerCommand("shoot", self.shootCommand)
        NamedCommands.registerCommand("intakeTrans", self.drivetrain.intakeCommand)
        

        autos = ["Dispense", "testAuto", "Dispense2"]
        auto = PathPlannerAuto(autos[DriverStation.getLocation()-1])
        
        #auto = PathPlannerAuto("testAuto")

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
        self.drivetrain.intake.intakeDrive.set(1)
        self.shooter.feedNote()

    def stage2(self):
        self.drivetrain.intake.intakeDrive.set(-.7)
        self.shooter.feedNote()
       

    def stage3(self):
        self.drivetrain.intake.intakeDrive.set(0)
        self.shooter.feedMotor.set(1)
  
    def stage4(self):
        self.shooter.feedMotor.set(0)
        self.drivetrain.intake.intakeDrive.set(0)


    def end(self):
        self.shooter.stopFlywheels()
        self.shooter.feedMotor.set(0)

    def systemTempCheck(self):
        motorControllers = [
            self.drivetrain.frontLeftDrive,
            self.drivetrain.frontRightDrive,
            self.drivetrain.backLeftDrive,
            self.drivetrain.backRightDrive,
            self.drivetrain.backLeftRotation,
            self.drivetrain.backRightRotation,
            self.drivetrain.frontLeftRotation,
            self.drivetrain.frontRightRotation,
            self.drivetrain.intake.intakeRotation,
            self.drivetrain.intake.intakeDrive,
            self.shooter.shooterDrive1,
            self.shooter.shooterDrive2,
            self.shooter.feedMotor,
            self.shooter.rotationMotor,
            self.climber.leftClimber,
            self.climber.rightClimber
        ]

        burntFlag = False
        for motorController in motorControllers:
            temp = motorController.getMotorTemperature()
            if temp > 90:
                print(f"[x] Motor {motorController.getDeviceId()}, {temp}C")
                burntFlag = True
            else:
                print(f"[-] Motor {motorController.getDeviceId()}, {temp}C")
        

        if burntFlag:
            for i in range(100):
                print("!!! ---------------- MOTORS TOO HOT ------------------- !!!")

        #print(f"Motor {motorController.getDeviceId()}, {temp}C")

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.drivetrain.resetHarder()   
        #self.drivetrain.gyro.set_yaw(0)
        self.transferStartTime = 0

        self.xboxController = wpilib.XboxController(1)
        self.joystick = wpilib.Joystick(0)

        self.systemTempCheck()
        self.transferCommand.schedule()

    def autoAim(self):
        kP = .02 #.013
        kI = .006
        tx = self.limey.getLimey()["tx"]
        if tx == 0:
            return -1
        
        arbFF = .023
        self.shooterInte += .02*tx

        return kP * (tx) + arbFF * (tx/abs(tx)) + kI * self.shooterInte


    def shooterAim(self):
        distance = self.limey.getTarget()["distance"]
        angle = self.limey.getTarget()["angle"]

        # if lost tracking, dont change shooter position for smoother operation
        if distance == 0:
            return 0

        # --- if long range ---
        #if distance > 2:
        #    self.shooter.setRot(getInterpAng(distance))

        # --- if close range ---
        # limelight angle is above the horizontal
        angle += 3.6
        angle = 90 - angle

        # angle measured from the top of the shooter to vertical, 0 degrees is up, 90 is horizontal
        # angle to rotations of motor
        rot = 0.210843373494 * angle - 6.45180722892
        self.shooter.setRot(rot-1) #round(rot-1, 1)

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
        #smd
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


        heading = self.drivetrain.getPose().rotation()
        if self.drivetrain.shouldFlipPath():
            # flip towards driver pespective if on red side
            heading = heading.rotateBy(Rotation2d(math.pi))

        heading = heading.radians()


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
        #self.drivetrain.intake.rotateDown()
            
        if self.xboxController.getXButtonReleased():
            self.transferCommand.schedule()

        #if self.drivetrain.intake.intakeDrive.getOutputCurrent() > 50:
        #    self.transferCommand.schedule()

        if self.xboxController.getXButtonPressed() and self.transferCommand.isScheduled():
            self.transferCommand.cancel()

        self.drivetrain.intake.intakeControllerUpdate()


        if self.xboxController.getYButton():
            self.drivetrain.intake.intakeRotation.set(1)
            return 1
        
        if self.xboxController.getYButtonReleased():
            self.drivetrain.intake.intakeRotation.set(0)
            self.drivetrain.intake.intakeHomeSetpoint = self.drivetrain.intake.intakeRotEnc.getPosition()
            self.drivetrain.intake.intakeDownSetpoint = self.drivetrain.intake.intakeRotEnc.getPosition() - 28



        # VERY SMART FIX
        if not self.transferCommand.isScheduled() :
            # climber stuff
            if self.xboxController.getLeftStickButton():
                self.shooter.targetAmp()

            # other stuff
            intakeButton = self.xboxController.getXButton()     


            self.drivetrain.intake.intakeDrive.set(0)
            self.shooter.stopFlywheels()


            if intakeButton: # if it is currently held
                self.drivetrain.intake.rotateDown()

            elif not intakeButton:
                self.drivetrain.intake.rotateHome()


            if self.globalTimer.getMatchTime() or True:
                if self.xboxController.getRightStickButton():
                    self.climber.setUp()
                
                else:
                    self.climber.rest()
            

            if self.xboxController.getLeftTriggerAxis() > .5:
                #self.shooter.targetSpeaker()
                self.shooter.spinFlywheels()
                self.shooterAim()

            if self.xboxController.getLeftTriggerAxis() < .4:
                self.shooterInte = 0


            if self.xboxController.getRightTriggerAxis() > .5:
                if self.shooter.getSpeed() or self.xboxController.getLeftBumper() or self.xboxController.getRightBumper():
                    self.shooter.feedNote()
                    self.drivetrain.intake.intakeDrive.set(-1)
                
            else:
                self.shooter.resetFeed()

            if self.xboxController.getLeftBumper():
                self.shooter.targetAmp()
                self.shooter.spinFlywheels()

            if self.xboxController.getRightBumper():
                self.shooter.setRot(14)

            if not self.xboxController.getLeftStickButton() and not self.xboxController.getRightStickButton() and not self.xboxController.getRightBumper() and not self.xboxController.getLeftBumper() and (self.xboxController.getLeftTriggerAxis() < .5) and not self.joystick.getTrigger():
                self.shooter.goHome()






        #self.drivetrain.intake.stopMotors()
        if self.xboxController.getAButton():
            self.drivetrain.intake.intakeDrive.set(1)

        if self.joystick.getRawButtonPressed(3):
            if self.drivetrain.shouldFlipPath():
                self.drivetrain.gyro.set_yaw(180)
            else:
                self.drivetrain.gyro.set_yaw(0)






        



if __name__ == "__main__":
    wpilib.run(MyRobot)

