package frc.robot.controls.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;

public class TankControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;
    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    public TankControls(DifferentialDriveEx drive, Joystick leftJoystick, Joystick rightJoystick) {
        m_drive = drive;
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;
        addHid(m_leftJoystick, m_rightJoystick);
    }

    @Override
    public void update() {
        m_drive.tankDrive(m_leftJoystick.getY(), m_rightJoystick.getY());
    }

    @Override
    public Class<? extends DriveControlBase> changeRequested() {
        return null;
    }

    @Override
    public String getName() {
        return "Curvature Controls";
    }

    static {
        DriveControlBase.addControlSystem(TankControls.class);
    }
}