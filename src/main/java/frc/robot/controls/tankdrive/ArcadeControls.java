package frc.robot.controls.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;
import frc.robot.hardware.HardwareMap;

public class ArcadeControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;
    private Joystick m_joystick;

    public ArcadeControls(DifferentialDriveEx drive, Joystick joystick) {
        m_drive = drive;
        m_joystick = joystick;
        addHid(m_joystick);
    }

    @Override
    public void update() {
        m_drive.arcadeDrive(m_joystick.getY(), -m_joystick.getX()/2);
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