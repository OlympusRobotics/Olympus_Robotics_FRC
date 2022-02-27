#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsControlModule.h>
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        //motors
        TalonSRX frontLeftMotor = {HardwareIDs::fLM};
        TalonSRX backLeftMotor = {HardwareIDs::bLM};
        TalonSRX frontRightMotor = {HardwareIDs::fRM};
        TalonSRX backRightMotor = {HardwareIDs::bRM};
        
        //controllers
        //these have to be set differently since they're going thru the frc namespace
        frc::Joystick rightJoystick{HardwareIDs::rJ};
        frc::Joystick leftJoystick{HardwareIDs::lJ};
        frc::XboxController xBox{HardwareIDs::xBox};

        // pnematics or however u spell it
        frc::DoubleSolenoid grabberSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
};