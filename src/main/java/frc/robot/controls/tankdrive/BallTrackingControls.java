package frc.robot.controls.tankdrive;

import frc.robot.hardware.HardwareMap;
import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;

public class BallTrackingControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;
    private static final double kPositionMax = 320;
    private static final double kMultiplierTolerance = .99;
    private int m_position;
    private double m_speedMultiplier;
    private double m_speed;
    private int m_radius;

    public BallTrackingControls(DifferentialDriveEx drive) {
        m_drive = drive;
    }

    @Override
    public void update() {
        if (m_radius != 0) {
            m_speed = Math.sqrt((1.0/(Math.abs(m_radius))));
        }
        else {
            m_speed = 0.0;
        }
        m_speedMultiplier = m_speed * (m_position/kPositionMax);
        //System.out.println(m_speedMultiplier + ", " + m_speed + ", " + m_position + ", " + m_radius);
        if (1.0 - m_speedMultiplier < kMultiplierTolerance) {
            m_drive.tankDrive(-m_speed, -(m_speed - m_speedMultiplier));
        }
        else if (Math.abs(-1.0 - m_speedMultiplier) < kMultiplierTolerance) {
            m_drive.tankDrive(-(m_speed + m_speedMultiplier), -m_speed);
        }
        else {
            m_drive.tankDrive(-m_speed, -m_speed);
        }
    }

    public void setDisplacement(int pos) {
        m_position = pos;
    }

    public void setRadius(int radius) {
        m_radius = radius;
    }

    @Override
    public Class<? extends DriveControlBase> changeRequested() {
        if (!(HardwareMap.kLeftJoystick.getTrigger())) {
            return ArcadeControls.class;
        }
        return null;
    }

    @Override
    public String getName() {
        return "Ball Tracking Controls";
    }

    static {
        DriveControlBase.addControlSystem(BallTrackingControls.class);
    }
}