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
#from wpilib.cameraserver import CameraServer
from cscore import CameraServer
from wpilib import SmartDashboard
import logging

class MyRobot(commands2.TimedCommandRobot):

    def startRumble(self):
        self.xboxController.setRumble(self.xboxController.RumbleType.kRightRumble,1)
        self.xboxController.setRumble(self.xboxController.RumbleType.kLeftRumble,1)

    def stopRumble(self):
        self.xboxController.setRumble(self.xboxController.RumbleType.kRightRumble,0)
        self.xboxController.setRumble(self.xboxController.RumbleType.kLeftRumble,0)


    def robotInit(self):
        """"""
        self.threeNote = "testAuto"
        self.center2Auto = "center2Auto"
        self.centerRun = "centerAuto"
        self.twoNote = "twoNote"
        self.preload = "preload"
        self.limeyTest = "limeyTest"
        self.stage3Note = "stage3Note2"
        self.stage4Note = "4notetest"
        self.disrupt = "disruptAll"
        self.shootCenter = "shootCenter"
        self.ampSide3Note = "ampSideCenter3"
        self.chooser = wpilib.SendableChooser()

        
        self.chooser.setDefaultOption("Three Note", self.threeNote)
        self.chooser.addOption("2 Note amp side, shoot center line note", self.ampSide3Note)
        self.chooser.addOption("Two note, amp side", self.center2Auto)
        self.chooser.addOption("Shoot and run source side", self.centerRun)
        self.chooser.addOption("Two note, in front of speaker", self.twoNote)
        self.chooser.addOption("Shoot preload", self.preload)
        self.chooser.addOption("Limey test", self.limeyTest)
        self.chooser.addOption("stage 3 note", self.stage3Note)
        self.chooser.addOption("disrupt", self.disrupt)
        self.chooser.addOption("4 note", self.stage4Note)
        self.chooser.addOption("go to center from source, pass center notes", self.shootCenter)
        SmartDashboard.putData("Auto choices", self.chooser)


        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        CameraServer.startAutomaticCapture()

        self.drivetrain = drivetrain.DriveTrain()
        self.climber = climber.Climber()
        self.shooter = shooter.Shooter()
        self.limey = Limey.Limey()
        
        self.configure_auto()
        self.globalTimer = wpilib.Timer()
        self.globalTimer.start()


        self.transferCommand = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.shooter.goHome, self),
            commands2.InstantCommand(self.shooter.feedNote, self),
            commands2.InstantCommand(self.drivetrain.intake.rotateHome, self),
            #commands2.InstantCommand(self.shooter.goHome, self),
            #commands2.InstantCommand(self.drivetrain.intake.transferHome, self),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(.4), self),
            commands2.WaitUntilCommand(self.drivetrain.intake.isHomePos),
            #commands2.WaitCommand(.2),
            commands2.InstantCommand(lambda: self.stage1(-1), self),
            #commands2.WaitCommand(.75),
            commands2.WaitCommand(1),
            commands2.InstantCommand(lambda: self.stage2(-0.1), self),
            commands2.WaitUntilCommand(lambda: not self.shooter.shooterSensor.get()).withTimeout(1.5),
            #commands2.WaitCommand(0),
            commands2.InstantCommand(self.end, self),
            commands2.InstantCommand(self.startRumble, self),
            commands2.WaitCommand(.2),
            commands2.InstantCommand(self.stopRumble, self)
        )

        self.intakeHomeCommand = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.shooter.spinFlywheels, self),
            commands2.InstantCommand(self.shooter.feedNote, self),
            commands2.InstantCommand(self.drivetrain.intake.rotateHome, self),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(.4), self),
            commands2.WaitUntilCommand(self.drivetrain.intake.isHomePos),
            commands2.InstantCommand(self.stage1, self),
            commands2.WaitCommand(1),
            commands2.InstantCommand(self.stage2, self),
            commands2.WaitCommand(1),

        )

        self.intakeEject = commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.drivetrain.intake.rotateEject, self),
            commands2.WaitUntilCommand(self.drivetrain.intake.isEjectPos),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(-1), self),
            commands2.WaitCommand(.5),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(0), self),
        )

        

        self.shooterot = 0
        self.shooterInte = 0

        self.shooterPackets = 0
        self.aimGyroAngle = 0

        


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
        #self.gyro.set_yaw(0)
        # Load the path you want to follow using its name in the GUI
        #self.shootNamedCommand = self.runOnce(self.shootCommand)
        self.shootCommandIntakeDelay = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.PrintCommand("SHOOTER RAN"),
            commands2.WaitCommand(.7),
            commands2.PrintCommand("FEED NOTE"),
            commands2.cmd.runOnce( self.shooter.feedNote),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(-1)),
            commands2.WaitCommand(.5),
            commands2.PrintCommand("FLYWHEEL COMMAND"),
            commands2.cmd.runOnce(self.shooter.stopFlywheels),
            commands2.cmd.runOnce(lambda: self.shooter.feedMotor.set(0)),
            commands2.PrintCommand("                            SHOOT COMMAND RAN"),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(1)),
            commands2.cmd.runOnce(self.drivetrain.intake.rotateDown),
            commands2.WaitCommand(.2499)
        )

        self.shootCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.PrintCommand("SHOOTER RAN"),
            commands2.WaitCommand(.7),
            commands2.PrintCommand("FEED NOTE"),
            commands2.cmd.runOnce( self.shooter.feedNote),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(-1)),
            commands2.WaitCommand(.5),
            commands2.PrintCommand("FLYWHEEL COMMAND"),
            commands2.cmd.runOnce(self.shooter.stopFlywheels),
            commands2.cmd.runOnce(lambda: self.shooter.feedMotor.set(0)),
            commands2.PrintCommand("                            SHOOT COMMAND RAN"),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(1)),
            commands2.cmd.runOnce(self.drivetrain.intake.rotateDown),
        )

        self.justShootCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.PrintCommand("SHOOTER RAN"),
            commands2.WaitCommand(.7),
            commands2.PrintCommand("FEED NOTE"),
            commands2.cmd.runOnce( self.shooter.feedNote),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(-1)),
            commands2.WaitCommand(.5),
            commands2.PrintCommand("FLYWHEEL COMMAND"),
            commands2.cmd.runOnce(self.shooter.stopFlywheels),
            commands2.cmd.runOnce(lambda: self.shooter.feedMotor.set(0)),
            commands2.PrintCommand("                            SHOOT COMMAND RAN"),
        )
        """
        
                commands2.cmd.runOnce(self.shooter.stopFlywheels()),
                commands2.cmd.runOnce(self.shooter.feedMotor.set(0)),
        """

        self.drivetrain.intakeCommand = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
            commands2.cmd.runOnce(self.shooter.feedNote),
            commands2.cmd.runOnce(self.drivetrain.intake.rotateHome),
            commands2.cmd.runOnce(self.shooter.goHome),
            commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(1)),
            commands2.WaitCommand(1.3),
        )



        self.aimAndShoot = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()), 
            
            commands2.InstantCommand(self.shooter.goHome, self),
            commands2.InstantCommand(self.shooter.feedNote, self),
            commands2.cmd.runOnce(self.shooter.spinFlywheels),
            commands2.InstantCommand(self.drivetrain.intake.rotateHome, self),
            #commands2.InstantCommand(self.shooter.goHome, self),
            #commands2.InstantCommand(self.drivetrain.intake.transferHome, self),
            commands2.InstantCommand(lambda: self.drivetrain.intake.intakeDrive.set(.4), self),
            commands2.WaitUntilCommand(self.drivetrain.intake.isHomePos),
            #commands2.WaitCommand(.2),
            commands2.InstantCommand(lambda: self.stage1(-1), self),
            #commands2.WaitCommand(.75),
            commands2.WaitCommand(1),
            commands2.InstantCommand(lambda: self.stage2(-0.1), self),
            commands2.WaitUntilCommand(lambda: not self.shooter.shooterSensor.get()).withTimeout(1.5),
            #commands2.WaitCommand(0),
            commands2.InstantCommand(self.end, self),

            commands2.cmd.runOnce(self.shooterAim),
            commands2.WaitCommand(1.6),
            commands2.cmd.runOnce(self.shooter.feedNote)
        )

        self.stopWheels = commands2.SequentialCommandGroup(
            commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
        )

        #NamedCommands.registerCommand("shoot", commands2.InstantCommand(lambda: self.shootCommand.schedule(), self))
        #NamedCommands.registerCommand("intakeTrans", commands2.InstantCommand(lambda: self.drivetrain.intakeCommand.schedule(), self))
        
        NamedCommands.registerCommand("shoot", self.shootCommand)
        NamedCommands.registerCommand("intakeTrans", self.drivetrain.intakeCommand)
        NamedCommands.registerCommand("justShoot", self.justShootCommand)
        NamedCommands.registerCommand("aimAndShoot", self.aimAndShoot)
        NamedCommands.registerCommand("stopWheels", self.stopWheels)
        NamedCommands.registerCommand("shootIntakeDelay", self.shootCommandIntakeDelay)
        

        #autos = ["Dispense", "testAuto", "Dispense2"]
        #auto = PathPlannerAuto(autos[DriverStation.getLocation()-1])
        #auto = PathPlannerAuto("centerAuto")
        self.autoSelected = self.chooser.getSelected()
        auto = PathPlannerAuto(self.autoSelected)

        return auto

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

        three = True

        if three:
            self.command = self.getAutonomousCommand()
            if self.command:
                commands2.SequentialCommandGroup(
                    self.command,
                    commands2.cmd.runOnce(lambda: self.drivetrain.stopMotors()),
                ).schedule()
                #self.command.schedule()

            

        else:
            self.shootCommand = commands2.SequentialCommandGroup(
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
                commands2.cmd.runOnce(lambda: self.drivetrain.intake.intakeDrive.set(0)),
            )

            self.shootCommand.schedule()
        
        

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.drivetrain.updateOdometry()
        # my bad

    
    def stage1(self, power=-1):
        self.drivetrain.intake.intakeDrive.set(1)
        self.shooter.feedNote(power)

    def stage2(self, power=-1):
        self.drivetrain.intake.intakeDrive.set(-.7)
        self.shooter.feedNote(power)
       

    def stage3(self):
        self.drivetrain.intake.intakeDrive.set(0)
        self.shooter.feedMotor.set(1)
  
    def stage4(self):
        self.shooter.feedMotor.set(0)
        self.drivetrain.intake.intakeDrive.set(0)


    def end(self):
        self.drivetrain.intake.intakeDrive.set(0)
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

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        self.drivetrain.resetHarder()   
        #self.drivetrain.gyro.set_yaw(0)
        self.transferStartTime = 0

        self.xboxController = wpilib.XboxController(1)
        self.joystick = wpilib.Joystick(0)

        self.systemTempCheck()

    def autoAim(self):
        """Poll limelight occasionally and pid with gyro angle"""
        kP = -.01
        kI = -.005

        
        arbFF = .023
        if self.limey.getHorizTarget() == -1:
            pass
        else:
            self.aimGyroAngle = 360 * (self.limey.getHorizTarget() / (2 * math.pi)) + self.drivetrain.gyro.get_yaw().value_as_double

        #print(self.aimGyroAngle)
        error = ((self.drivetrain.gyro.get_yaw().value_as_double)-self.aimGyroAngle)
        
        P = error * kP 
        self.shooterInte += .02*error
        
        FF = 0
        if P != 0:
            FF = (P/abs(P)) * arbFF
        return P + FF + kI * self.shooterInte


    def fautoAim(self):
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

        #self.drivetrain.shouldUpdateIntakeController = False
        """This function is called periodically during teleoperated mode."""
        
        # ----------------------- DRIVETRAIN CODE -----------------------
        #if not wpilib.DriverStation.getJoystickIsXbox(0):
        
        xspeed = self.joystick.getX()
        yspeed = -self.joystick.getY()
        tspeed = self.joystick.getTwist()

        if self.xboxController.getLeftY() > .5:
            self.shooter.targetSpeakerFromStage()
            self.shooter.spinFlyAnal(1)



        if self.joystick.getTrigger() and not self.xboxController.getLeftY() > .5:
            tspeed = self.autoAim()
            if tspeed == -1:
                tspeed = self.joystick.getTwist()
        elif not self.joystick.getTrigger():
            self.shooterInte = 0

            


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
        if self.xboxController.getYButton():
            self.shooter.spinFlyAnal(-1)
            #self.shooter.targetSource()
            return 0
        
        if self.xboxController.getAButtonPressed():
            self.intakeEject.schedule()    
        
        if self.xboxController.getBButton():
            self.drivetrain.intake.rotateDown()
            self.drivetrain.shouldUpdateIntakeController = True
            return 0

        if self.xboxController.getBButtonReleased():
            self.intakeHomeCommand.schedule()
            #elf.drivetrain.intake.intakeDrive.set(-1) 

        #if self.intakeHomeCommand.isScheduled():
        #    return 0  
        
        #if self.xboxController.getYButton():
        #    self.drivetrain.intake.intakeRotation.set(.4)
        #    self.drivetrain.shouldUpdateIntakeController = False
        #    return 0
        
        if self.xboxController.getXButton():
            #self.drivetrain.intake.intakeRotation.set(-.4)
            #self.drivetrain.intake.intakeDrive.set(1)
            self.drivetrain.intake.rotateDown()
            self.drivetrain.shouldUpdateIntakeController = True 

            return 0
        

        # maybe rmeove???
        self.drivetrain.intake.intakeRotation.set(0)

        #self.drivetrain.intake.rotateDown()

        
        if self.xboxController.getXButtonPressed() and self.transferCommand.isScheduled():
            self.transferCommand.cancel()

        if self.shooter.isUp2Speed():
            self.startRumble()
        elif not self.transferCommand.isScheduled():
            self.stopRumble()

        if self.xboxController.getXButtonReleased():
            self.transferCommand.schedule()

        self.drivetrain.intake.intakeControllerUpdate()

        if self.xboxController.getLeftTriggerAxis() > .1:
            self.shooter.spinFlyAnal(self.xboxController.getLeftTriggerAxis()**2)

            if self.joystick.getTrigger() and not self.transferCommand.isScheduled():
                self.shooterAim()

        if self.xboxController.getLeftTriggerAxis() < .1 and not self.xboxController.getLeftY() > .5:
            self.shooter.spinFlyAnal(0)
        
        # VERY SMART FIX
        if not self.transferCommand.isScheduled() and not self.intakeHomeCommand.isScheduled() and not self.intakeEject.isScheduled():
            # climber stuff, need to put shooter in up pos for climbing
            if self.xboxController.getLeftStickButton():
                self.shooter.targetAmp()

            # other stuff
            intakeButton = self.xboxController.getXButton()     


            self.drivetrain.intake.intakeDrive.set(0)
            


            if intakeButton: # if it is currently held
                self.drivetrain.intake.rotateDown()
                

            elif not intakeButton and not self.xboxController.getAButton():
                self.drivetrain.intake.rotateHome()
            
            """
            if self.joystick.getRawButton(1):
                self.climber.leftClimber.set(-.1)
                self.climber.rightClimber.set(-.1)
            
            else:
                if self.joystick.getRawButtonReleased(1):
                    leftLowerPos = self.climber.lClimberRotEnc.getPosition()
                    rightLowerPos = self.climber.rClimberRotEnc.getPosition()

                    self.climber.LeftfullyExtended += leftLowerPos
                    self.climber.RightfullyExtended += rightLowerPos

                    self.climber.leftResting = leftLowerPos
                    self.climber.rightResting = rightLowerPos
            """
            if self.xboxController.getRightStickButton():
                self.climber.setUp()
                #self.shooter.targetAmp()
                
            else:
                self.climber.rest()
                
            


            if self.xboxController.getRightTriggerAxis() > .5:
                if self.shooter.getSpeed() or self.xboxController.getLeftBumper() or self.xboxController.getRightBumper():
                    self.shooter.feedNote()
                    self.drivetrain.intake.intakeDrive.set(-1)
                
            else:
                self.shooter.resetFeed()

            if self.xboxController.getLeftBumper():
                self.shooter.targetAmp()

            if self.xboxController.getRightBumper():
                self.shooter.setRot(14)

            if not self.xboxController.getLeftStickButton() and not self.xboxController.getRightStickButton() and not self.xboxController.getRightBumper() and not self.xboxController.getLeftBumper() and (self.xboxController.getLeftTriggerAxis() < .5) and not self.joystick.getTrigger() and not self.xboxController.getYButton() and not self.xboxController.getLeftY() > .5:
                self.shooter.goHome()

        #self.drivetrain.intake.stopMotors()
        #if self.xboxController.getAButton():
        #    self.drivetrain.intake.intakeDrive.set(1)

        # Reset gyuro pos 
        if self.joystick.getRawButtonPressed(3):
            self.drivetrain.gyro.set_yaw(0)

if __name__ == "__main__":
    #logging.setLoggerClass
    logger = logging.getLogger()
    logger.propagate = False
    wpilib.run(MyRobot)

