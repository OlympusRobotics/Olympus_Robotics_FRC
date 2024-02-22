import commands2
import rev


class Shooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # motor init
        self.shooterDrive1 = rev.CANSparkMax(102, rev.CANSparkMax.MotorType.kBrushless)
        self.shooterDrive2 = rev.CANSparkMax(103, rev.CANSparkMax.MotorType.kBrushless)
        self.feedMotor = rev.CANSparkMax(104, rev.CANSparkMax.MotorType.kBrushless)

        self.shooterController1 = self.shooterDrive1.getPIDController()
        self.shooterController2 = self.shooterDrive2.getPIDController()

        self.shooterDriveEnc1 = self.shooterDrive1.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)
        self.shooterDriveEnc2 = self.shooterDrive2.getEncoder(rev.SparkRelativeEncoder.Type.kHallSensor, 42)


        self.configShooterMotor(self.shooterController1)
        self.configShooterMotor(self.shooterController2)


        # intake rotation motor
        self.rotationMotor = rev.CANSparkMax(105, rev.CANSparkMax.MotorType.kBrushless)
        self.rotationMotor = self.rotationMotor.getPIDController()

        # intake global variables
        self.shooterMaxVelocity = 1000
        self.homeSetpoint = 0 # home position in rotations
        self.shootSetpoint = 10 # shoot position in rotations
        self.ampSetpoint = 10 # amp position in rotations

    def configShooterMotor(self, motor: rev.SparkPIDController):
        # Neo PID constants
        kP = 0.1
        kI = 1e-4
        kD = 1
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
    
    def feedNote(self):
        self.feedMotor.set(.2)

    def stopFeed(self):
        self.feedMotor.set(0)

    def stopFlywheels(self):
        self.shooterController1.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        self.shooterController2.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
        