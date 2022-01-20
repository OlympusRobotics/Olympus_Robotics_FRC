package frc.robot.controls.mecanumdrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.MecanumDriveEx;
import frc.robot.hardware.JoystickMap;

public class JoystickCartesianControls extends DriveControlBase {
    private MecanumDriveEx m_mecanumDrive;
    private Joystick m_joystick;
    private ADXRS450_Gyro m_gyro;
    private boolean m_brake;
    
    public JoystickCartesianControls(MecanumDriveEx mecanumDrive, Joystick joystick, ADXRS450_Gyro gyro) {
        m_mecanumDrive = mecanumDrive;
        m_joystick = joystick;
        m_gyro = gyro;
        addHid(m_joystick);
    }

    @Override
    public void update() {
        if (m_joystick.getRawButton(JoystickMap.kTrigger.value)) {
            m_mecanumDrive.driveCartesian(m_joystick.getX(), m_joystick.getY(), m_joystick.getZ());
        } else {
            m_mecanumDrive.driveCartesian(m_joystick.getX(), m_joystick.getY(), m_joystick.getZ(), m_gyro.getAngle());
        }
        if (m_joystick.getRawButtonPressed(JoystickMap.kStickFront.value)) {
            for (SpeedController motor : m_mecanumDrive.motorList) {
                if (motor instanceof WPI_TalonSRX) {
                    ((WPI_TalonSRX) motor).setNeutralMode(m_brake ? NeutralMode.Brake : NeutralMode.Coast);
                }
            }
            m_brake = !m_brake;
        }
    }

    @Override
    public Class<? extends DriveControlBase> changeRequested() {
        return null;
    }

    @Override
    public String getName() {
        return "Joystick Cartesian Controls";
    }

    static {
        DriveControlBase.addControlSystem(JoystickCartesianControls.class);
    }
}