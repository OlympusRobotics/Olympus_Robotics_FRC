
package frc.robot.controls;

import edu.wpi.first.wpilibj.Solenoid;
public class PistonControl {
    private static Solenoid m_piston;


    public PistonControl(Solenoid piston) {
        m_piston = piston;
        m_piston.set(false);
    }
    public static void set(boolean value) {
        m_piston.set(value);
    }

    public boolean get() {
        return m_piston.get();
    }
}

