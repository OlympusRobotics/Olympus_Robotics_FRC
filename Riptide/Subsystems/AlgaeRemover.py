import wpilib
import rev
import commands2

class algaeRemover(commands2.Subsystem):
    def __init__(self):
        #Motor Initialization
        self.armRotationMotor = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)

        #Encoder Initialization
        self.armRotationEncoder = self.armRotationMotor.getEncoder()

        #Algae remover arm Configuration
        armRotationConfig = rev.SparkMaxConfig()
        armRotationConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        armRotationConfig.smartCurrentLimit(30)

        armPIDConfig = armRotationConfig.closedLoop
        armPIDConfig.pidf(0.01, 0, 0, 0.002, rev.ClosedLoopSlot.kSlot0)
        armPIDConfig.FeedbackSensor.kPrimaryEncoder

        MAXMotionConfig = armPIDConfig.maxMotion
        MAXMotionConfig.maxVelocity(3000)
        MAXMotionConfig.maxAcceleration(5000)
        MAXMotionConfig.allowedClosedLoopError(0.1)

        self.armRotationMotor.configure(armRotationConfig, self.armRotationMotor.ResetMode.kResetSafeParameters, self.armRotationMotor.PersistMode.kPersistParameters)
        
        #Closed Loop Configuration
        self.armClosedLoop = self.armRotationMotor.getClosedLoopController()

        #Arm Positions
        self.homePosition = 0
        self.readyPosition = 8
        self.Position1 = 6.5
        self.Position2 = 9.5

        super().__init__()
        
    def isTooHot(self):
        if (self.armRotationMotor.getMotorTemperature() > 90):
            return True
        else:
            return False

    def setHomePosition(self):
        self.armClosedLoop.setReference(self.homePosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setReadyPosition(self):
        self.armClosedLoop.setReference(self.readyPosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setPostion1(self):
        self.armClosedLoop.setReference(self.Position1, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setPostion2(self):
        self.armClosedLoop.setReference(self.Position2, self.armRotationMotor.ControlType.kMAXMotionPositionControl)


    def manualControl(self, input):
        self.armRotationMotor.setVoltage(input)

        