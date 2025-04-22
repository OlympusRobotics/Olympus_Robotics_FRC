import wpilib
import wpimath
import rev
import commands2
import wpimath.controller
import wpimath.trajectory
import math
import wpimath.units
from commands2 import PIDCommand

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
        self.intakeEncoder = wpilib.DutyCycleEncoder(2, 1, 0.216)
        self.intakeRotationEncoder = self.intakeMotor.getEncoder()
        self.armRotationEncoder = self.armRotationMotor.getEncoder()

        #Algae intake/outtake arm motor Configuration
        armRotationConfig = rev.SparkMaxConfig()
        armRotationConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        armRotationConfig.smartCurrentLimit(30)
        armRotationConfig.inverted(False)

        self.armRotationMotor.configure(armRotationConfig, self.armRotationMotor.ResetMode.kResetSafeParameters, self.armRotationMotor.PersistMode.kPersistParameters)
        
        #Closed Loop Configuration
        self.controller = wpimath.controller.PIDController(1.2, 0.0, 0.0)
        self.controller.setTolerance(0.05)
        
        #Arm Positions
        self.homePosition = 0
        self.algaeEjectPosition = .05
        self.intakePosition = .202

        super().__init__()
        
    def isTooHot(self):
        """ 
        Checks if the algae intake arm motors are getting too hot. This is useful to notify either the drive team or any other operator if the motors are becoming too hot.
        """
        if ((self.armRotationMotor.getMotorTemperature() > 90) or (self.intakeMotor.getMotorTemperature() > 90)):
            return True
        else:
            return False
        
    def getPosition(self):
        """ 
        Gets the position of the encoder. Calling this method rounds the encoder value to three decimal places for less encoder noise.
        """
        return round(self.intakeEncoder.get(), 3)
        
    def setPosition(self, NewPosition: str):
        """ 
        This function continously runs to keep the algae arm in a fixed position. The PID controller always needs to be run for the arm to maintain the position.
        """
        if (NewPosition == "Intake"):
            self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.intakePosition))
        elif (NewPosition == "Home"):
            self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.homePosition))
        elif (NewPosition == "Eject"):
            self.armRotationMotor.set(self.controller.calculate(self.intakeEncoder.get(), self.algaeEjectPosition))

    def intake(self):
        """ 
        This runs the motor at 35% full speed to intake the algae.
        """
        self.intakeMotor.set(.55)
        
    def stopIntakeMotor(self):
        """ 
        Stops the intake motor from running.
        """
        self.intakeMotor.stopMotor()
        
    def algaeEject(self):
        """ 
        Spins the intake motor in reverse at 30% speed to spit out the algae.
        """
        self.intakeMotor.set(-.3)

    def algaeCheck(self):
        """ 
        This method pulls current from the motor. If it detects a change in current that is greater than 30 amps, it will return true.
        When the algae gets pulled in by the intake motor, there is a current spike. We use this to gauge whether or not an algae ball has been pulled in.
        """
        if (self.intakeMotor.getOutputCurrent() > 30) and (self.intakeRotationEncoder.getVelocity() < 2500):
            return True
        else:
            return False

    def manualControl(self, input):
        """ 
        This method manually controls the position of the arm. It should never be used unless the automations somehow fail.
        """
        self.armRotationMotor.setVoltage(input)
