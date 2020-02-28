package frc.robot.controls;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PistonControl {
    private DoubleSolenoid m_piston;

    public PistonControl(DoubleSolenoid piston) {
        m_piston = piston;
        m_piston.set(DoubleSolenoid.Value.kOff);
    }
    public void forward() {
        m_piston.set(DoubleSolenoid.Value.kForward);
    }
    public void reverse() {
        m_piston.set(DoubleSolenoid.Value.kReverse);
    }
    public void off() {
        m_piston.set(DoubleSolenoid.Value.kOff);
    }
    public DoubleSolenoid.Value getState() {
        return m_piston.get();
    }
}