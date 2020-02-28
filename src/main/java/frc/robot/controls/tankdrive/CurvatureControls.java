package frc.robot.controls.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;
import frc.robot.hardware.JoystickMap;

public class CurvatureControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;
    private Joystick m_joystick;

    public CurvatureControls(DifferentialDriveEx drive, Joystick joystick) {
        m_drive = drive;
        m_joystick = joystick;
        addHid(m_joystick);
    }

    @Override
    public void update() {
        m_drive.curvatureDrive(m_joystick.getY(), m_joystick.getX(), m_joystick.getRawButton(JoystickMap.kTrigger.value));
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
        DriveControlBase.addControlSystem(CurvatureControls.class);
    }
}