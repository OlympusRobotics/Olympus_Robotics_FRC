package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controls.DoubleJoystick;
import frc.robot.controls.PistonControl;
import frc.robot.drive.DifferentialDriveEx;

/**
 * Hardware Map
 */
public class HardwareMap {
    public static final WPI_TalonSRX kIntakeMotor = new WPI_TalonSRX(HardwareIDs.kIntakeMotor.value);
    public static final WPI_TalonSRX kOutputMotor1 = new WPI_TalonSRX(HardwareIDs.kOutputMotor1.value);
    public static final WPI_TalonSRX kOutputMotor2 = new WPI_TalonSRX(HardwareIDs.kOutputMotor2.value);
    public static final WPI_TalonSRX kBeltMotor = new WPI_TalonSRX(HardwareIDs.kBeltMotor.value);
    /**private static final WPI_VictorSPX kFrontLeftMotor = new WPI_VictorSPX(HardwareIDs.kFrontLeftMotor.value);
    private static final WPI_VictorSPX kFrontRightMotor = new WPI_VictorSPX(HardwareIDs.kFrontRightMotor.value);
    private static final WPI_VictorSPX kRearLeftMotor = new WPI_VictorSPX(HardwareIDs.kRearLeftMotor.value);
    private static final WPI_VictorSPX kRearRightMotor = new WPI_VictorSPX(HardwareIDs.kRearRightMotor.value);**/
    public static final WPI_VictorSPX kFrontLeftMotor = new WPI_VictorSPX(HardwareIDs.kFrontLeftMotor.value);
    public static final WPI_VictorSPX kFrontRightMotor = new WPI_VictorSPX(HardwareIDs.kFrontRightMotor.value);
    public static final WPI_VictorSPX kRearLeftMotor = new WPI_VictorSPX(HardwareIDs.kRearLeftMotor.value);
    public static final WPI_VictorSPX kRearRightMotor = new WPI_VictorSPX(HardwareIDs.kRearRightMotor.value);
    public static final Joystick kLeftJoystick = new Joystick(HardwareIDs.kLeftJoystick.value);
    public static final Joystick kRightJoystick = new Joystick(HardwareIDs.kRightJoystick.value);
    public static final DoubleJoystick kDoubleJoystick = new DoubleJoystick(kLeftJoystick, kRightJoystick);
    public static final XboxController kXbox = new XboxController(HardwareIDs.kController.value);
    public static final DifferentialDriveEx kDrive = new DifferentialDriveEx(kFrontLeftMotor, kRearLeftMotor, kFrontRightMotor, kRearRightMotor);
    private static final Solenoid kSolenoid = new Solenoid(0);
    /**private static final Solenoid kBarSolenoid = new Solenoid(0);
    public static final PistonControl kBarPiston = new PistonControl(kBarSolenoid);**/
    public static final PistonControl kPiston = new PistonControl(kSolenoid);

    static {
        kFrontLeftMotor.setInverted(InvertType.InvertMotorOutput);
        kFrontRightMotor.setInverted(InvertType.InvertMotorOutput);
        kRearLeftMotor.setInverted(InvertType.InvertMotorOutput);
        kRearRightMotor.setInverted(InvertType.InvertMotorOutput);
        kIntakeMotor.setInverted(InvertType.InvertMotorOutput);
    }
}