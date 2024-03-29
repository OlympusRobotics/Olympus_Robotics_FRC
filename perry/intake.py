import commands2
import rev
from helper import tempProt
import wpimath
import wpimath.controller
import wpimath.trajectory
import time
import logging
import wpilib
import random

class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # motor init
        self.intakeDrive = rev.CANSparkMax(16, rev.CANSparkMax.MotorType.kBrushless)
        self.intakeRotation = rev.CANSparkMax(15, rev.CANSparkMax.MotorType.kBrushless)

        self.intakeController = self.intakeRotation.getPIDController()
        self.intakeRotEnc = self.intakeRotation.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.shaftEnc = wpilib.Encoder(1,2)
        
        # Neo PID constants
        kP = 0.022
        kI = 0
        kD = 0
        kIz = 0 
        kFF = 0 
        kMaxOutput = .4
        kMinOutput = -.4

        # set PID constants
        self.intakeController.setP(kP)
        self.intakeController.setI(kI)
        self.intakeController.setD(kD)
        self.intakeController.setIZone(kIz)
        self.intakeController.setFF(kFF)
        self.intakeController.setOutputRange(kMinOutput, kMaxOutput)

        # intake global variables
        self.intakeHomeSetpoint = 0 # top position in rotations (not enc values) for the top position of the intake
        self.intakeDownSetpoint = 1072 #.567 # rotations for the bottom position of the intake
        self.ejectSetpoint = 600

        # intake motion profiling
        self.kDt = .02
        self.constraints = wpimath.trajectory.TrapezoidProfile.Constraints(50000, 100000)
        self.controller = wpimath.controller.PIDController(
            .00055, 0, 0.00, self.kDt
        ) #.022

        self.shaftEnc.reset()

        #self.controller.enableContinuousInput(0,1)
        


    def periodic(self) -> None:
        if random.random() > 0.5:
            logging.debug(f"shaft enc value - {self.shaftEnc.getDistance()}")


    def intakeControllerUpdate(self):
        rotPower = self.controller.calculate(self.shaftEnc.getDistance())
        self.intakeRotation.set(-rotPower)


    def intakeTempProt(self):
        return tempProt(self.intakeRotation)

    def rotateHome(self):
        if self.intakeTempProt() > 0:
            return 1

        # set refernce changes the setpoint - rotations
        #self.intakeController.setReference(self.intakeHomeSetpoint, rev.CANSparkMax.ControlType.kPosition)

        self.controller.setSetpoint(self.intakeHomeSetpoint)
        #self.controller.setGoal(self.intakeHomeSetpoint)
        self.intakeDrive.set(0)

    def rotateEject(self):
        if self.intakeTempProt() > 0:
            return 1

        # set refernce changes the setpoint - rotations
        #self.intakeController.setReference(self.intakeHomeSetpoint, rev.CANSparkMax.ControlType.kPosition)
        self.controller.setSetpoint(self.ejectSetpoint)
        self.intakeDrive.set(0)

    def rotateDown(self):
        if self.intakeTempProt() > 0:
            return 1
        
        #self.intakeController.setReference(self.intakeDownSetpoint, rev.CANSparkMax.ControlType.kPosition)
        self.controller.setSetpoint(self.intakeDownSetpoint)
        self.intakeDrive.set(1)



    def moveUp(self):
        self.intakeRotation.set(1)

    
    def isHomePos(self):
        if abs(self.shaftEnc.getDistance() - self.intakeHomeSetpoint) < 30:
            return True

        return False
    
    def isEjectPos(self):
        if abs(self.shaftEnc.getDistance() - self.ejectSetpoint) < 30:
            return True

        return False
    
    