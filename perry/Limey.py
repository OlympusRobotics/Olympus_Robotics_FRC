import commands2
import wpilib
import ntcore
import math
import wpilib.drive
import rev
import phoenix5 as ctre
from wpimath.geometry import Translation2d, Rotation2d, Pose2d



class Limey(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")
    def getLimey(self) -> tuple[float, float, float]:

        self.tx = self.table.getEntry("tx").getDouble(-1000)
        self.ty = self.table.getEntry("ty").getDouble(-1000)
        self.ta = self.table.getEntry("ta").getDouble(-1000)

        return (self.tx, self.ty, self.ta)
    def getPose(self):
        poes = self.table.getEntry("BotPoseBlue").getDoubleArray([])
        print(poes) 
        if len(poes) == 0:
            return Pose2d()
        return Pose2d(Translation2d(poes[0], poes[1]), Rotation2d(poes[-1]))
    
