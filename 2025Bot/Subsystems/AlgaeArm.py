import wpilib
import rev
import commands2

class algaeArm(commands2.Subsystem):
    def __init__(self):
        #Motor Initialization
        self.armRotationMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)
        self.intakeMotor = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)

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
        self.intakePosition = 20

        super().__init__()
        
    def isIntakePos(self):
        if (self.armRotationEncoder.getPosition() >= 19.0 and self.armRotationEncoder.getPosition() <= 22.0):
            return True
        else:
            return False
        
    def isTooHot(self):
        if ((self.armRotationMotor.getMotorTemperature() > 90) or (self.intakeMotor.getMotorTemperature() > 90)):
            return True
        else:
            return False

    def setHomePosition(self):
        self.armClosedLoop.setReference(self.homePosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def setIntakePosition(self):
        self.armClosedLoop.setReference(self.intakePosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def intake(self):
        self.intakeMotor.set(.2)
        
    def stopIntakeMotor(self):
        self.intakeMotor.stopMotor()
        
    def algaeEject(self):
        self.intakeMotor.set(-.2)

    def manualControl(self, input):
        self.armRotationMotor.setVoltage(input)

        