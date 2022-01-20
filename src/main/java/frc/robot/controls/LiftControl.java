package frc.robot.controls;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;

//import edu.wpi.first.wpilibj.PIDController;

public class LiftControl {
    private SpeedController m_lift;
    public Encoder m_encoder;
    @SuppressWarnings("removal")
    private PIDController m_pidController;
    private double m_target;
    private static final double Ku = -0.03;
    private static final double Tu = .23;
    private static final double Kp = .6*Ku;
    private static final double Ki = 1.2*Ku/Tu;
    private static final double Kd = 3*Ku*Tu/40;
    
    public LiftControl(SpeedController lift, Encoder encoder) {
        m_lift = lift;
        /**
        m_encoder = encoder;
        m_encoder.reset();
        m_target = m_encoder.get();
        m_pidController = new PIDController(Kp, Ki, Kd, m_encoder, m_lift);
        m_pidController.setAbsoluteTolerance(1);
        m_pidController.setSetpoint(m_target);
        m_pidController.enable();
        **/
    }
    /**
    public void increase(double increment) {
        System.out.println(m_encoder.get());
        System.out.println(m_target);
        m_target += increment;
        m_pidController.setSetpoint(m_target);
    }

    public void decrease(double decrement) {
        System.out.println(m_encoder.get());
        System.out.println(m_target);
        m_target -= decrement;
        m_pidController.setSetpoint(m_target);        
    }

    public void enable() {
        m_pidController.enable();
    }

    public void disable() {
        m_pidController.disable();
    }

    public boolean onTarget() {
        return m_pidController.onTarget();
    }**/

    public void set(double speed) {
        m_lift.set(speed);
    }
}