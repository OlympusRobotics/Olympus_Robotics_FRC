package frc.robot.controls.tankdrive;

import frc.robot.controls.DoubleJoystick;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;

public class ArcadeControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;
    private DoubleJoystick m_joystick;

    public ArcadeControls(DifferentialDriveEx drive, DoubleJoystick joystick) {
        m_drive = drive;
        m_joystick = joystick;
        addHid(m_joystick.getHID());
    }

    @Override
    public void update() {
        m_drive.arcadeDrive(m_joystick.getY(), -m_joystick.getX()/1.5);
    }

    @Override
    public Class<? extends DriveControlBase> changeRequested() {
        /**if (HardwareMap.kLeftJoystick.getTrigger())
        {
            return BallTrackingControls.class;
        }**/
        return null;
    }

    @Override
    public String getName() {
        return "Arcade Controls";
    }

    static {
        DriveControlBase.addControlSystem(ArcadeControls.class, true);
    }
}