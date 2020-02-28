package frc.robot.hardware;

public enum HardwareIDs {
    kFrontLeftMotor(2),
    kFrontRightMotor(1),
    kRearLeftMotor(3),
    kRearRightMotor(4),
    kPulleyMotor1(5),
    kPulleyMotor2(6),
    kIntakeMotor(2),
    kOutputMotor1(0),
    kOutputMotor2(1),
    kBeltMotor(3),
    kCompressor(0),
    kLeftJoystick(1),
    kRightJoystick(2),
    kController(0);

    public final int value;

    HardwareIDs(int value) {
        this.value = value;
    }
}