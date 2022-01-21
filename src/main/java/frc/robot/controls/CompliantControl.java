package frc.robot.controls;

import edu.wpi.first.wpilibj.SpeedController;

public class CompliantControl {
    private SpeedController m_motor;
    
    public CompliantControl(SpeedController motor) {
        m_motor = motor;
    }

    public void forward(double speed) {
        m_motor.set(speed);
    }

    public void reverse(double speed) {
        m_motor.set(-speed);
    }

    public void neutral() {
        m_motor.set(0);
    }

}