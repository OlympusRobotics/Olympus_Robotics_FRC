from drivetrain import DriveTrain

class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = DriveTrain()
    
    def getAutoCommand(self):
        return self.drivetrain.getAutonomousCommand()