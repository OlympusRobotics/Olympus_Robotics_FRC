#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

namespace hw{
    
    // static TalonSRX R0 = {0};
    // static TalonSRX R1 = {1};
    // static TalonSRX R2 = {2};
    // static TalonSRX R3 = {3};

    // static TalonSRX D0 = {4};
    // static TalonSRX D1 = {5};
    // static TalonSRX D2 = {6};
    

    

   // static TalonSRX controllers[] = {1,2,3,4,5,8};
    // rotation motor
    static TalonSRX frontLeftRotation = {1};
    static TalonSRX frontRightRotation = {2};
    static TalonSRX backLeftRotation = {3};
    static TalonSRX backRightRotation = {4};

    // drive motor
    static TalonSRX frontLeftDrive = {5};
    static VictorSPX frontRightDrive = {6};
    static VictorSPX backLeftDrive = {7};
    static TalonSRX backRightDrive = {8};
    static int test = 0;

    // controllers
    static frc::Joystick rightJoystick{0};
    static frc::XboxController xBox{1};

    //static std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

}   