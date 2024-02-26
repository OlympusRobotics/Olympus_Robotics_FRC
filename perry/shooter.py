import commands2
import rev


class Shooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # motor init
        self.shooterDrive1 = rev.CANSparkMax(102, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterDrive2 = rev.CANSparkMax(103, rev.CANSparkMax.MotorType.kBrushless)
        self.feedMotor = rev.CANSparkMax(104, rev.CANSparkMax.MotorType.kBrushless)
        # shooter rotation motor
        self.rotationMotor = rev.CANSparkMax(105, rev.CANSparkMax.MotorType.kBrushless)
        self.rotationMotor = self.rotationMotor.getPIDController()

        self.shooterController1 = self.shooterDrive1.getPIDController()
        self.shooterController2 = self.shooterDrive2.getPIDController()
        self.feedController = self.feedMotor.getPIDController()

        self.shooterDriveEnc1 = self.shooterDrive1.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.shooterDriveEnc2 = self.shooterDrive2.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.feedMotorEnc = self.feedMotor.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)


        self.configShooterMotor(self.shooterController1)
        self.configShooterMotor(self.shooterController2)
        self.configFeedMotor(self.feedController)

        # shooter global variables
        self.shooterMaxVelocity = 1000
        self.homeSetpoint = 0 # home position in rotations
        self.shootSetpoint = 10 # shoot position in rotations
        self.ampSetpoint = 10 # amp position in rotations
        self.grabSetpoint = 1.5 # how many rotations it takes to grab onto the note

    def configFeedMotor(self, motor: rev.SparkPIDController):
        kP = 0.5
        kI = 1e-4
        kD = 0
        kIz = 0 
        kFF = 0 
        kMaxOutput = 1 
        kMinOutput = -1

        # set PID constants
        motor.setP(kP)
        motor.setI(kI)
        motor.setD(kD)
        motor.setIZone(kIz)
        motor.setFF(kFF)
        motor.setOutputRange(kMinOutput, kMaxOutput)

    def configShooterMotor(self, motor: rev.SparkPIDController):
        # Neo PID constants
        kP = 0.1
        kI = 1e-4
        kD = 0
        kIz = 0 
        kFF = 0 
        kMaxOutput = 1 
        kMinOutput = -1

        # set PID constants
        motor.setP(kP)
        motor.setI(kI)
        motor.setD(kD)
        motor.setIZone(kIz)
        motor.setFF(kFF)
        motor.setOutputRange(kMinOutput, kMaxOutput)
    
    def spinFlywheels(self) -> bool:
        self.shooterController1.setReference(self.shooterMaxVelocity, rev.CANSparkMax.ControlType.kVelocity)
        self.shooterController2.setReference(-self.shooterMaxVelocity, rev.CANSparkMax.ControlType.kVelocity)

        error1 = abs(abs(self.shooterDriveEnc1.getVelocity())-abs(self.shooterMaxVelocity))
        error2 = abs(abs(self.shooterDriveEnc2.getVelocity())-abs(self.shooterMaxVelocity))
    
        if (error1 + error2 > 100):
            return False
        
        return True
    
    def grabNote(self):
        self.shooterController2.setReference(self.grabSetpoint, rev.CANSparkMax.ControlType.kPosition)
        if abs(self.shooterDriveEnc2.getPosition() - self.grabSetpoint) < .1: # if in position
            return True
        return False

    def resetFeed(self):
        self.feedMotor.set(0)
        # reset encoder
        self.feedMotorEnc.setPosition(0)

    def stopFlywheels(self):
        self.shooterController1.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.shooterController2.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        
    def feedNote(self):
        self.feedMotor.set(.6)
