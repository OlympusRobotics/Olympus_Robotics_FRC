import commands2
import rev
from helper import tempProt

class Intake(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # motor init
        self.intakeDrive = rev.CANSparkMax(16, rev.CANSparkMax.MotorType.kBrushless)
        self.intakeRotation = rev.CANSparkMax(15, rev.CANSparkMax.MotorType.kBrushless)

        self.intakeController = self.intakeRotation.getPIDController()
        self.intakeRotEnc = self.intakeRotation.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

        # Neo PID constants
        kP = 0.1
        kI = 0
        kD = 0
        kIz = 0 
        kFF = 0 
        kMaxOutput = 1 
        kMinOutput = -1

        # set PID constants
        self.intakeController.setP(kP)
        self.intakeController.setI(kI)
        self.intakeController.setD(kD)
        self.intakeController.setIZone(kIz)
        self.intakeController.setFF(kFF)
        self.intakeController.setOutputRange(kMinOutput, kMaxOutput)

        # intake global variables
        self.intakeHomeSetpoint = 0 # top position in rotations (not enc values) for the top position of the intake
        self.intakeDownSetpoint = -8.5 # rotations for the bottom position of the intake


    def intakeTempProt(self):
        return tempProt(self.intakeRotation)

    def rotateHome(self):
        if self.intakeTempProt() > 0:
            return 1

        # set refernce changes the setpoint - rotations
        self.intakeController.setReference(self.intakeHomeSetpoint, rev.CANSparkMax.ControlType.kPosition)
        self.intakeDrive.set(0)

    def rotateDown(self):
        if self.intakeTempProt() > 0:
            return 1
        
        self.intakeController.setReference(self.intakeDownSetpoint, rev.CANSparkMax.ControlType.kPosition)
        self.intakeDrive.set(1)


    def stopMotors(self):
        self.intakeRotation.set(0)