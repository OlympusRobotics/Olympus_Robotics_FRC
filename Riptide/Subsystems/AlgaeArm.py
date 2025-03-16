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
        armRotationConfig.inverted(False)

        armPIDConfig = armRotationConfig.closedLoop
        armPIDConfig.pid(0.011, 0.000002, 0.0009, rev.ClosedLoopSlot.kSlot0)
        armPIDConfig.FeedbackSensor.kPrimaryEncoder

        MAXMotionConfig = armPIDConfig.maxMotion
        MAXMotionConfig.maxVelocity(6000)
        MAXMotionConfig.maxAcceleration(6000)
        MAXMotionConfig.allowedClosedLoopError(0.5)

        self.armRotationMotor.configure(armRotationConfig, self.armRotationMotor.ResetMode.kResetSafeParameters, self.armRotationMotor.PersistMode.kPersistParameters)
        
        #Closed Loop Configuration
        self.armClosedLoop = self.armRotationMotor.getClosedLoopController()

        #Arm Positions
        self.homePosition = 0
        self.algaeEjectPosition = 3.5
        self.intakePosition = 9

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

    def setEjectPosition(self):
        self.armClosedLoop.setReference(self.algaeEjectPosition, self.armRotationMotor.ControlType.kMAXMotionPositionControl)

    def intake(self):
        self.intakeMotor.set(.3)
        
    def stopIntakeMotor(self):
        self.intakeMotor.stopMotor()
        
    def algaeEject(self):
        self.intakeMotor.set(-.2)

    def algaeCheck(self):
        if self.intakeMotor.getOutputCurrent() > 30:
            return True
        else:
            return False

    def manualControl(self, input):
        self.armRotationMotor.setVoltage(input)

        