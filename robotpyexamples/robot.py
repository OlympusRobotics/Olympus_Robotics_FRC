
import wpilib
import wpilib.drive
import rev
import phoenix6 as ctre
import wpimath 
from wpimath import controller

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called pon program startup and
        should be used for any initialization code.
        """
        self.fuckyouhenry = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.leftEnc = ctre.hardware.CANcoder(11)
        

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""


    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

       
        self.leftPID = controller.PIDController(4.2,0,.0001)
        self.leftPID.enableContinuousInput(-.5,.5)
        self.leftPID.setSetpoint(0.0)
        print("HENRYH WHY")
        

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        #self.fuckyouhenry.set(1.0)
        self.fuckyouhenry.set(-self.leftPID.calculate(self.leftEnc.get_absolute_position()._value, 5.0))

        


if __name__ == "__main__":
    wpilib.run(MyRobot)