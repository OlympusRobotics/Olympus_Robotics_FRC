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
#include <frc/AnalogInput.h>
#include "rev/CANSparkMax.h"
#include "Robot.h"
#include "frc/DoubleSolenoid.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"

namespace hw{
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
    
    static frc::Joystick rightJoystick{0};
    static frc::XboxController xBox{1};

    // controllers
    




    static std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    

    
    

    //static frc::AnalogInput pins{0};
    //static frc::Compressor compressor{0, frc::PneumaticsModuleType::CTREPCM};

}   
