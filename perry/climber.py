import commands2
import rev

class Climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        #Motor init
        self.leftClimber = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.rightClimber = rev.CANSparkMax(20, rev.CANSparkMax.MotorType.kBrushless)

        self.lClimberController = self.leftClimber.getPIDController()
        self.rClimberController = self.rightClimber.getPIDController()

        self.lClimberRotEnc = self.leftClimber.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.rClimberRotEnc = self.rightClimber.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)

        # Neo PID constants
        kP = 0.5
        kI = 0
        kD = 0
        kIz = 0 
        kFF = 0 
        kMaxOutput = 1 
        kMinOutput = -1

        # set PID constants
        self.lClimberController.setP(kP)
        self.lClimberController.setI(kI)
        self.lClimberController.setD(kD)
        self.lClimberController.setIZone(kIz)
        self.lClimberController.setFF(kFF)
        self.lClimberController.setOutputRange(kMinOutput, kMaxOutput)

        self.rClimberController.setP(kP)
        self.rClimberController.setI(kI)
        self.rClimberController.setD(kD)
        self.rClimberController.setIZone(kIz)
        self.rClimberController.setFF(kFF)
        self.rClimberController.setOutputRange(kMinOutput, kMaxOutput)

        #ClimberSetpoint
        self.restingPoint = -67
        self.fullyExtended = 0

    def setUp(self):
        self.lClimberController.setReference(self.fullyExtended, rev.CANSparkMax.ControlType.kPosition)
        self.rClimberController.setReference(self.fullyExtended, rev.CANSparkMax.ControlType.kPosition)
        #self.leftClimber.set(.1)
        #self.rightClimber.set(.1)

    def rest(self):
        self.lClimberController.setReference(self.restingPoint, rev.CANSparkMax.ControlType.kPosition)
        self.rClimberController.setReference(self.restingPoint, rev.CANSparkMax.ControlType.kPosition)
        #self.leftClimber.set(-.1)
        #self.rightClimber.set(-.1)

    """def stopMotors(self):
        pass
        #self.leftClimber.set(0)
        #self.rightClimber.set(0)"""
        
    



