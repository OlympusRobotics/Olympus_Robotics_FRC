package frc.robot.controls.mecanumdrive;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controls.DriveControlBase;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DancepadControls extends DriveControlBase {
    private MecanumDrive m_mecanumDrive;
    private GenericHID m_dancepad;
    public double speedMultiplier = 0.5;

    public DancepadControls(MecanumDrive mecanumDrive, GenericHID dancepad) {
        m_mecanumDrive = mecanumDrive;
        m_dancepad = dancepad;
    }

    @Override
    public void update() {
        int angle = m_dancepad.getPOV();
        m_mecanumDrive.drivePolar(speedMultiplier, angle > 180 ? 360 - angle : -angle, 0);
    }

    @Override
    public Class<? extends DriveControlBase> changeRequested() {
        return null;
    }

    @Override
    public String getName() {
        return "DDR Dancepad Controls";
    }

    static {
        DriveControlBase.addControlSystem(DancepadControls.class);
    }
}