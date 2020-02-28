package frc.robot.controls.tankdrive;

import frc.robot.controls.DriveControlBase;
import frc.robot.drive.DifferentialDriveEx;
import frc.robot.hardware.HardwareMap;

public class NewBallTrackingControls extends DriveControlBase {
    private DifferentialDriveEx m_drive;

    private static final double MaxRadius = 500;
    private static final double MinRadius = 25;
    private static final double MaxOutR = 1;
    private static final double MinOutR = -1;
    private static final double NeutralOffSetR= 0.5;
    private static final double PGainR = 0.5;

    private static final double MaxX = 480;
    private static final double MinX = 10;
    private static final double Maxx = 1;
    private static final double Minx= -1;
    private static final double NeutralOffSetX= 0;
    private static final double PGainX = 0.25;

    private double ScaledX = 0;
    private double ScaledRadius = 0;
    private double X;
    private double Radius;

    public NewBallTrackingControls(DifferentialDriveEx drive) {
        m_drive = drive;
    }

    @Override
    public void update() {
        ScaledX =  PGainX * ((((Maxx - Minx)*((X - MinX + 240)/(MaxX - MinX))) + Minx) -  NeutralOffSetX);
        ScaledRadius =  PGainR * ((((MaxOutR - MinOutR)*((Radius - MinRadius)/(MaxRadius - MinRadius))) + MinOutR) -  NeutralOffSetR);
        m_drive.tankDrive(ScaledRadius - ScaledX, ScaledRadius + ScaledX);
    }

    public void setDisplacement(int displacement) {
        X = displacement;
    }

    public void setRadius(int radius) {
        Radius = radius;
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
        return "New Ball Tracking Controls";
    }
}