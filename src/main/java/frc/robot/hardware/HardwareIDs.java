package frc.robot.hardware;

public enum HardwareIDs {
    kFrontLeftMotor(0),
    kFrontRightMotor(1),
    kRearLeftMotor(2),
    kRearRightMotor(3),
    kIntakeMotor(4),
    kOutputMotor1(5),
    kOutputMotor2(6),
    kBeltMotor(7),
    kCompressor(0),
    kLeftJoystick(1),
    kRightJoystick(0),
    kController(2);

    public final int value;

    HardwareIDs(int value) {
        this.value = value;
    }
}