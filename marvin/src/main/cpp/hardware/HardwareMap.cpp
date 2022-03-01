#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        //motors
        TalonSRX frontLeftMotor = {HardwareIDs::fLM};
        TalonSRX backLeftMotor = {HardwareIDs::bLM};
        TalonSRX frontRightMotor = {HardwareIDs::fRM};
        TalonSRX backRightMotor = {HardwareIDs::bRM};
        TalonSRX leftArmMotor = {HardwareIDs::lAM};
        TalonSRX rightArmMotor = {HardwareIDs::rAM};
        //TalonSRX testMotor = {-1};
        
        //controllers
        //these have to be set differently since they're going thru the frc namespace
        frc::Joystick rightJoystick{HardwareIDs::rJ};
        frc::Joystick leftJoystick{HardwareIDs::lJ};
        frc::XboxController xBox{HardwareIDs::xBox};

        //pnuematics
        frc::DoubleSolenoid bigArmSolenoid{HardwareIDs::pcm, frc::PneumaticsModuleType::CTREPCM, 0, 1}; //climbing arm pistons
        frc::DoubleSolenoid intakeSolenoid{HardwareIDs::pcm, frc::PneumaticsModuleType::CTREPCM, 2, 3}; //ball grabber pistons

};