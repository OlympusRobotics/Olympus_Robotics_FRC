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

        #Config Elevator motor ID 10
        leaderConfig = rev.SparkMaxConfig()
        brake = leaderConfig.setIdleMode(idleMode=rev.SparkMaxConfig.IdleMode.kBrake)
        limit = leaderConfig.smartCurrentLimit(30)
        
        leaderConfig.apply(brake)
        leaderConfig.apply(limit)
        
        self.elevatorMoveMotor1.configure(leaderConfig, self.elevatorMoveMotor1.ResetMode.kResetSafeParameters, self.elevatorMoveMotor1.PersistMode.kPersistParameters)     

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


    def setL1(self):
        self.goal = TrapezoidProfile.State(15, 0)
        
        calculatePID = self.PIDcontroller.calculate(self.elevatorEncoder.getPosition(), self.goal.position)
        calculateFF = self.feedForward.calculate(rpm2rps(self.elevatorEncoder.getVelocity()), self.goal.velocity)
        
        self.elevatorMoveMotor1.setVoltage(calculatePID + calculateFF)
        self.elevatorMoveMotor2.setVoltage(calculatePID + calculateFF)
        
    def setL2(self):
        self.goal = TrapezoidProfile.State(35, 0)
        
        calculatePID = self.PIDcontroller.calculate(self.elevatorEncoder.getPosition(), self.goal.position)
        calculateFF = self.feedForward.calculate(rpm2rps(self.elevatorEncoder.getVelocity()), self.goal.velocity)
        
        self.elevatorMoveMotor1.setVoltage(calculatePID + calculateFF)
        self.elevatorMoveMotor2.setVoltage(calculatePID + calculateFF) #Figure out sparkMAX Follower mode

    def manualControl(self, input):
        self.elevatorMoveMotor1.set(input)
        self.elevatorMoveMotor2.set(input)