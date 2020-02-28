package frc.robot.hardware;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Hardware Map
 */
public class HardwareMap {
    public static final WPI_TalonSRX kIntakeMotor = new WPI_TalonSRX(HardwareIDs.kIntakeMotor.value);
    public static final WPI_TalonSRX kOutputMotor1 = new WPI_TalonSRX(HardwareIDs.kOutputMotor1.value);
    public static final WPI_TalonSRX kOutputMotor2 = new WPI_TalonSRX(HardwareIDs.kOutputMotor2.value);
    public static final WPI_TalonSRX kBeltMotor = new WPI_TalonSRX(HardwareIDs.kBeltMotor.value);
    //private static final WPI_VictorSPX kFrontLeftMotor = new WPI_VictorSPX(HardwareIDs.kFrontLeftMotor.value);
    //private static final WPI_VictorSPX kFrontRightMotor = new WPI_VictorSPX(HardwareIDs.kFrontRightMotor.value);
    //private static final WPI_TalonSRX kRearLeftMotor = new WPI_TalonSRX(HardwareIDs.kRearLeftMotor.value);
    //private static final WPI_VictorSPX kRearRightMotor = new WPI_VictorSPX(HardwareIDs.kRearRightMotor.value);
    //private static final WPI_TalonSRX kPulleyMotor1 = new WPI_TalonSRX(HardwareIDs.kPulleyMotor1.value);
    //private static final WPI_TalonSRX kPulleyMotor2 = new WPI_TalonSRX(HardwareIDs.kPulleyMotor2.value);
    //private static final WPI_TalonSRX kCompliantMotor = new WPI_TalonSRX(HardwareIDs.kIntakeMotor.value);
    //private static final SpeedControllerGroup kPulleyMotor = new SpeedControllerGroup(kPulleyMotor1, kPulleyMotor2);
    //private static final Encoder kEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    public static final Joystick kLeftJoystick = new Joystick(HardwareIDs.kLeftJoystick.value);
    public static final Joystick kRightJoystick = new Joystick(HardwareIDs.kRightJoystick.value);
    public static final XboxController kXbox = new XboxController(HardwareIDs.kController.value);
    //public static final LiftControl kLift = new LiftControl(kPulleyMotor, kEncoder);
    //public static final CompliantControl kCompliant = new CompliantControl(kCompliantMotor);
    //public static final DifferentialDriveEx kDrive = new DifferentialDriveEx(kFrontLeftMotor, kRearLeftMotor, kFrontRightMotor, kRearRightMotor);
    //private static final DoubleSolenoid kSolenoid = new DoubleSolenoid(0, 1);
    //public static final PistonControl kPiston = new PistonControl(kSolenoid);
    static {
        //kFrontLeftMotor.setInverted(InvertType.InvertMotorOutput);
        //kFrontRightMotor.setInverted(InvertType.InvertMotorOutput);
        //kRearLeftMotor.setInverted(InvertType.InvertMotorOutput);
        //kRearRightMotor.setInverted(InvertType.InvertMotorOutput);
        kIntakeMotor.setInverted(InvertType.InvertMotorOutput);
    }
}