import wpilib
import wpimath
import rev
import commands2
import wpimath.controller
import wpimath.trajectory
import math
import wpimath.units

def enc2Rad(EncoderInput: float):
    return EncoderInput * 2 * math.pi

def rpm2RadPerSec(MotorVelocity: float):
    return wpimath.units.rotationsPerMinuteToRadiansPerSecond(MotorVelocity)

class algaeArm(commands2.Subsystem):
    def __init__(self):
        #Motor Initialization
        self.armRotationMotor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)
        self.intakeMotor = rev.SparkMax(14, rev.SparkMax.MotorType.kBrushless)

        #Encoder Initialization
        self.intakeEncoder = wpilib.DutyCycleEncoder(2, 1, 0.223)
        self.armRotationEncoder = self.armRotationMotor.getEncoder()

        self.intake

        #Algae intake/outtake arm motor Configuration
        armRotationConfig = rev.SparkMaxConfig()
        armRotationConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        armRotationConfig.smartCurrentLimit(30)
        armRotationConfig.inverted(False)

        """ armPIDConfig = armRotationConfig.closedLoop
        armPIDConfig.pid(0.011, 0.000002, 0.0009, rev.ClosedLoopSlot.kSlot0)
        armPIDConfig.FeedbackSensor.kPrimaryEncoder

        MAXMotionConfig = armPIDConfig.maxMotion
        MAXMotionConfig.maxVelocity(6000)
        MAXMotionConfig.maxAcceleration(6000)
        MAXMotionConfig.allowedClosedLoopError(0.5) """

        self.armRotationMotor.configure(armRotationConfig, self.armRotationMotor.ResetMode.kResetSafeParameters, self.armRotationMotor.PersistMode.kPersistParameters)
        
        #Closed Loop Configuration
        #self.armClosedLoop = self.armRotationMotor.getClosedLoopController()
        constraints = wpimath.trajectory.TrapezoidProfile.Constraints(2000, 2000)
        self.controller = wpimath.controller.PIDController(1, 0.0, 0.0)
        self.controller.setSetpoint(0)
        self.feedForward = wpimath.controller.ArmFeedforward(0.0, 0.0, 0.0)
        self.controller.setTolerance(0.08)
        
        #Arm Positions
        self.homePosition = 0
        self.algaeEjectPosition = .05
        self.intakePosition = .212

        super().__init__()
        
    def isTooHot(self):
        if ((self.armRotationMotor.getMotorTemperature() > 90) or (self.intakeMotor.getMotorTemperature() > 90)):
            return True
        else:
            return False
        
    def getPosition(self):
        return self.intakeEncoder.get()
    
    def setPosition(self, NewPosition: float):
        self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), NewPosition))


    def setHomePosition(self):
        self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.homePosition))
        #print(position)

    def setIntakePosition(self):
        self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.intakePosition))


        print(self.controller.calculate(self.intakeEncoder.get(), self.intakePosition))

    def setEjectPosition(self):
        self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.algaeEjectPosition))
        print(self.getPosition())
        
        #print(position)

    def intake(self):
        self.intakeMotor.set(.5)
        
    def stopIntakeMotor(self):
        self.intakeMotor.stopMotor()
        
    def algaeEject(self):
        self.intakeMotor.set(-.3)

    def algaeCheck(self):
        if self.intakeMotor.getOutputCurrent() > 30:
            return True
        else:
            return False

    def manualControl(self, input):
        self.armRotationMotor.setVoltage(input)

        