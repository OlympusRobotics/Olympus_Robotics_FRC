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

        self.irSensor = wpilib.DigitalInput(0)

        self.elevatorEncoder1 = self.elevatorMoveMotor1.getEncoder()
        self.elevatorEncoder2 = self.elevatorMoveMotor2.getEncoder()

        #Config Elevator motor ID 10
        leaderConfig = rev.SparkMaxConfig()
        brake = leaderConfig.setIdleMode(idleMode=rev.SparkMaxConfig.IdleMode.kBrake)
        limit = leaderConfig.smartCurrentLimit(30)

        leaderConfig.closedLoop.pid(0.005, 0.0, 0.0, rev.ClosedLoopSlot.kSlot0)
        leaderConfig.closedLoop.outputRange(-1,1)
        leaderConfig.closedLoop.FeedbackSensor(rev.SparkMaxConfig().closedLoop.FeedbackSensor.kPrimaryEncoder)
        leaderConfig.closedLoop.maxMotion.maxAcceleration(1000).maxVelocity(1000).allowedClosedLoopError(0.4)

        
        self.elevatorMoveMotor1.configure(leaderConfig, self.elevatorMoveMotor1.ResetMode.kResetSafeParameters, self.elevatorMoveMotor1.PersistMode.kPersistParameters)     

        self.closedLoopController = self.elevatorMoveMotor1.getClosedLoopController()

        #Config Elevator motor ID 11
        followerConfig = rev.SparkMaxConfig()
        followerConfig.follow(10)
        followerConfig.apply(brake)
        followerConfig.apply(limit)

        self.elevatorMoveMotor2.configure(followerConfig, self.elevatorMoveMotor2.ResetMode.kResetSafeParameters, self.elevatorMoveMotor2.PersistMode.kPersistParameters)
        
        self.constraints = TrapezoidProfile.Constraints(1, 0.7)
        self.PIDcontroller = wpimath.controller.ProfiledPIDController(1, 0.0, 0.0, self.constraints)

        self.feedForward = wpimath.controller.ElevatorFeedforward(0.1, 0.02, 0.6) #Retune pls

        self.goal = TrapezoidProfile.State()
        self.setPoint = TrapezoidProfile.State()

        super().__init__()

    def setMAXmotion(self):
        self.closedLoopController.setReference(0, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)
        

    def setL1(self):       
        self.closedLoopController.setReference(15, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)
        
    def setL2(self):
        self.closedLoopController.setReference(35, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def setL3(self):
        self.closedLoopController.setReference(55, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def setL4(self):
        self.closedLoopController.setReference(75, self.elevatorMoveMotor1.ControlType.kMAXMotionPositionControl, rev.ClosedLoopSlot.kSlot0)

    def flyWheelSpin(self):
        self.outtakeMotor.set(1)
    
    def flyWheelStop(self):
        self.outtakeMotor.stopMotor()

    def coralCheck(self):
        return self.irSensor.get()

    def manualControl(self, input):
        self.elevatorMoveMotor1.setVoltage(input)