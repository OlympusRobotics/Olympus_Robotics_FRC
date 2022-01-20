package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class DoubleJoystick {
    private Joystick m_joystick1;
    private Joystick m_joystick2;

    public DoubleJoystick(Joystick joystick1, Joystick joystick2) {
        m_joystick1 = joystick1;
        m_joystick2 = joystick2;
    }

    public double getY() {
        if (m_joystick2.getY() < 0 || m_joystick1.getY() < 0) return Double.min(m_joystick1.getY(), m_joystick2.getY());
        return Double.max(m_joystick1.getY(), m_joystick2.getY());
    }

    public double getX() {
        if (m_joystick2.getX() < 0 || m_joystick1.getX() < 0) return Double.min(m_joystick1.getX(), m_joystick2.getX());
        return Double.max(m_joystick1.getX(), m_joystick2.getX());
    }

    public boolean getRawButton(int value) {
        return m_joystick1.getRawButton(value) || m_joystick2.getRawButton(value);
    }

    public GenericHID getHID() {
        return m_joystick1;
    }
}
