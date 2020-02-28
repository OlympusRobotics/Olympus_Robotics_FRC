package frc.robot.drive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DifferentialDriveEx extends DifferentialDrive implements IBrakeable {
    public SpeedController[] motorList;

    public DifferentialDriveEx(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(new SpeedControllerGroup(frontLeftMotor, rearLeftMotor),
                new SpeedControllerGroup(frontRightMotor, rearRightMotor));
        motorList = new SpeedController[] { frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor };
    }
}