package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;

public interface IBrakeable {
    default void brake(SpeedController... motors) {
        for (SpeedController motor : motors) {
            if (motor instanceof WPI_TalonSRX) {
                ((WPI_TalonSRX) motor).setNeutralMode(NeutralMode.Brake);
            } else if (motor instanceof WPI_VictorSPX) {
                ((WPI_VictorSPX) motor).setNeutralMode(NeutralMode.Brake);
            }
        }
    }
    
    default void coast(SpeedController... motors) {
        for (SpeedController motor : motors) {
            if (motor instanceof WPI_TalonSRX) {
                ((WPI_TalonSRX) motor).setNeutralMode(NeutralMode.Coast);
            } else if (motor instanceof WPI_VictorSPX) {
                ((WPI_VictorSPX) motor).setNeutralMode(NeutralMode.Coast);
            }
        }
    }
}