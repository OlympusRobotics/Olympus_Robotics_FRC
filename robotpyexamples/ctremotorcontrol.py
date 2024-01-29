import wpilib
import wpilib.drive
import ctre 
import rev


def driveCTRE(motor, speed) -> None:
    motor.set(ctre.ControlMode.PercentOutput, speed)

def drive(motor, speed) -> None:
    motor.set(speed)


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.motor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        self.motor1 = rev.CANSparkMax(13, rev.CANSparkMax.MotorType.kBrushless)
        self.motor2 = ctre.TalonSRX(8)
        self.motor3 = ctre.VictorSPX(7)
        self.joystick = wpilib.Joystick(0)


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""


    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        #self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())
        mag = self.joystick.getMagnitude()

        if self.joystick.getY() < 0:
            mag = -mag

        drive(self.motor, -mag)
        drive(self.motor1, mag)

        driveCTRE(self.motor2, mag)
        driveCTRE(self.motor3, mag)


if __name__ == "__main__":
    wpilib.run(MyRobot)
