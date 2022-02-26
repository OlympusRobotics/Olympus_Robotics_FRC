#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
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

        //pneumatics. For solenoids use the enum for the first port then enum +1 for the 2nd port 
        /*frc::PneumaticsControlModule pcm{0};
        frc::DoubleSolenoid grabberSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
        frc::Compressor compressor{0, frc::PneumaticsModuleType::CTREPCM};*/
};