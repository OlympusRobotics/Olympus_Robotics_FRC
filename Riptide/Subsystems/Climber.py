import wpilib
import rev
import commands2

class climber(commands2.Subsystem):
    def __init__(self):
        #Motor Initialization
        self.climberRotationMotor1 = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)
        self.climberRotationMotor2 = rev.SparkMax(16, rev.SparkMax.MotorType.kBrushless)

        #Encoder Initialization
        self.climberRotationEncoder = self.climberRotationMotor1.getEncoder()

        #climber motor 1 Configuration
        climberRotationConfig = rev.SparkMaxConfig()
        climberRotationConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        climberRotationConfig.smartCurrentLimit(30)

        climberPIDConfig = climberRotationConfig.closedLoop
        climberPIDConfig.pid(0.01, 0, 0, rev.ClosedLoopSlot.kSlot0)
        climberPIDConfig.FeedbackSensor.kPrimaryEncoder

        MAXMotionConfig = climberPIDConfig.maxMotion
        MAXMotionConfig.maxVelocity(1000)
        MAXMotionConfig.maxAcceleration(1000)
        MAXMotionConfig.allowedClosedLoopError(0.06)

        #climber motor 2 Configuration
        climberRotationConfig2 = rev.SparkMaxConfig()
        climberRotationConfig2.apply(climberRotationConfig)
        climberRotationConfig2.follow(15)

        self.climberRotationMotor1.configure(climberRotationConfig, self.climberRotationMotor1.ResetMode.kResetSafeParameters, self.climberRotationMotor1.PersistMode.kPersistParameters)
        self.climberRotationMotor2.configure(climberRotationConfig2, self.climberRotationMotor2.ResetMode.kResetSafeParameters, self.climberRotationMotor2.PersistMode.kPersistParameters)
    
        self.climberClosedLoop = self.climberRotationMotor1.getClosedLoopController()

        #climber Positions
        self.homePosition = 0
        self.climbPosition = 30

        super().__init__()

    def setHomePosition(self):
        self.climberClosedLoop.setReference(self.homePosition, self.climberRotationMotor1.ControlType.kMAXMotionPositionControl)

    def setClimbPosition(self):
        self.climberClosedLoop.setReference(self.climbPosition, self.climberRotationMotor1.ControlType.kMAXMotionPositionControl)

    def manualControl(self, input):
        self.climberRotationMotor1.setVoltage(input)

        
