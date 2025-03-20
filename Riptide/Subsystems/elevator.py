import rev
import math
import wpilib
import wpimath.units
import wpimath.controller
from commands2 import Subsystem
import wpimath.trajectory
from wpimath.trajectory import TrapezoidProfile

def rpm2rps(rpm):
    return rpm / 60


class Elevator(Subsystem):
    def __init__(self):

        #Motor init
        self.elevatorMoveMotor1 = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
        self.elevatorMoveMotor2 = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
        self.outtakeMotor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
        

        self.elevatorEncoder1 = self.elevatorMoveMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMoveMotor2.getEncoder()
        self.outtakeEncoder = self.outtakeMotor.getEncoder()

        #Config Elevator motor ID 10
        leaderConfig = rev.SparkMaxConfig()
        brake = leaderConfig.setIdleMode(idleMode=rev.SparkMaxConfig.IdleMode.kBrake)
        limit = leaderConfig.smartCurrentLimit(30)

        leaderConfig.closedLoop.pidf(0.009, 0.0, 0, 0.000053, rev.ClosedLoopSlot.kSlot0)
        leaderConfig.closedLoop.outputRange(-1,1)
        leaderConfig.closedLoop.FeedbackSensor(rev.SparkMaxConfig().closedLoop.FeedbackSensor.kPrimaryEncoder)
        leaderConfig.closedLoop.maxMotion.maxAcceleration(6000).maxVelocity(5000).allowedClosedLoopError(0.2)

        
        self.elevatorMoveMotor1.configure(leaderConfig, self.elevatorMoveMotor1.ResetMode.kResetSafeParameters, self.elevatorMoveMotor1.PersistMode.kPersistParameters)     

        self.closedLoopController = self.elevatorMoveMotor1.getClosedLoopController()

        #Config Elevator motor ID 11
        followerConfig = rev.SparkMaxConfig()
        followerConfig.follow(10)
        followerConfig.apply(brake)
        followerConfig.apply(limit)

        self.elevatorMoveMotor2.configure(followerConfig, self.elevatorMoveMotor2.ResetMode.kResetSafeParameters, self.elevatorMoveMotor2.PersistMode.kPersistParameters)

        self.homePos = 0
        self.intakePos = 30
        self.L1Pos =37
        self.L2Pos = 46
        self.L3Pos = 70
        super().__init__()
        
    def isTooHot(self):
        if ((self.elevatorMoveMotor1.getMotorTemperature() > 90) or (self.elevatorMoveMotor2.getMotorTemperature() > 90) or (self.outtakeMotor.getMotorTemperature() > 90)):
            return True
        else:
            return False

    def setHome(self):
        self.closedLoopController.setReference(self.homePos, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def setIntake(self):
        self.closedLoopController.setReference(self.intakePos, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def setL1(self):       
        self.closedLoopController.setReference(self.L1Pos,self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)
        
    def setL2(self):
        self.closedLoopController.setReference(self.L2Pos, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def setL3(self):
        self.closedLoopController.setReference(self.L3Pos, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    """ def coralCheck(self):         """
    """     return not self.irSensor.get() """
        
    def flyWheelSpin(self):
        self.outtakeMotor.set(-.3)
    
    def flyWheelStop(self):
        self.outtakeMotor.stopMotor()
    
    def coralUnstuck(self):
        self.outtakeMotor.set(.6)

    def manualControl(self, input):        
        self.elevatorMoveMotor1.setVoltage(input)