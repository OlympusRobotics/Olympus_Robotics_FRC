package frc.robot.hardware;

public enum JoystickMap {
    kTrigger(-1),
    kStickRight(-1),
    kStickLeft(-1),
    kStickFront(-1);

    public final int value;

    JoystickMap(int value) {
        this.value = value;
    }
}