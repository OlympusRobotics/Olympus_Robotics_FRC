package frc.robot.drive;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class MecanumDriveEx extends MecanumDrive implements IBrakeable {
    public SpeedController[] motorList;

    public MecanumDriveEx(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        motorList = new SpeedController[] {frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor};
    }
}