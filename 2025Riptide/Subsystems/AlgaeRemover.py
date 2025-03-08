import wpilib
import rev
import commands2

class algaeRemover(commands2.Subsystem):
    def __init__(self):
        #Motor Initialization
        self.armRotationMotor = rev.SparkMax(15, rev.SparkMax.MotorType.kBrushless)

        #Encoder Initialization
        self.armRotationEncoder = self.armRotationMotor.getEncoder()

        #Algae intake/outtake arm motor Configuration
        armRotationConfig = rev.SparkMaxConfig()
        armRotationConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        armRotationConfig.smartCurrentLimit(30)
        armRotationConfig.inverted(True)

        armPIDConfig = armRotationConfig.closedLoop
        armPIDConfig.pid(0.01, 0, 0, rev.ClosedLoopSlot.kSlot0)
        armPIDConfig.FeedbackSensor.kPrimaryEncoder

        MAXMotionConfig = armPIDConfig.maxMotion
        MAXMotionConfig.maxVelocity(1000)
        MAXMotionConfig.maxAcceleration(1000)
        MAXMotionConfig.allowedClosedLoopError(0.06)

        self.armRotationMotor.configure(armRotationConfig, self.armRotationMotor.ResetMode.kResetSafeParameters, self.armRotationMotor.PersistMode.kPersistParameters)
        
        #Closed Loop Configuration
        self.armClosedLoop = self.armRotationMotor.getClosedLoopController()

        #Arm Positions
        self.homePosition = 0
        self.Position1 = 20
        self.Position2 = 20

        0

        super().__init__()
        
    def isTooHot(self):
        if ((self.armRotationMotor.getMotorTemperature() > 90) or (self.intakeMotor.getMotorTemperature() > 90)):
            return True
        else:
            return False

    def setHomePosition(self):
        self.armClosedLoop.setReference(self.homePosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setPostion1(self):
        self.armClosedLoop.setReference(self.Position1, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setPostion2(self):
        self.armClosedLoop.setReference(self.Position2, self.armRotationMotor.ControlType.kMAXMotionPositionControl)


    def manualControl(self, input):
        self.armRotationMotor.setVoltage(input)

        