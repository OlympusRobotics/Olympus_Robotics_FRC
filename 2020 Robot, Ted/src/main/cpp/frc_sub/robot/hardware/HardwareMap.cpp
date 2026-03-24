#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h" //this one is the Talons and Victors (motor controllers)
#include "HardwareIDs.cpp" //this one is the values
#include "frc/PneumaticsBase.h"

class HardwareMap {
    public: //this needs to be public so everything can be accessed in other files
        
        //every value is pulled from the HardwareIDs file (they're just numbers)
        
        TalonSRX intakeMotor = {HardwareIDs::intakeMotor};
        TalonSRX outputMotor1 = {HardwareIDs::outputMotor1};
        TalonSRX outputMotor2 = {HardwareIDs::outputMotor2};
        TalonSRX beltMotor = {HardwareIDs::beltMotor};
        VictorSPX frontLeftMotor = {HardwareIDs::frontLeftMotor};
        VictorSPX frontRightMotor = {HardwareIDs::frontRightMotor};
        VictorSPX rearLeftMotor = {HardwareIDs::rearLeftMotor};
        VictorSPX rearRightMotor = {HardwareIDs::rearRightMotor};

        //these ones need to be initialized differently bc they're going through the frc thing
        frc::Joystick rightJoystick{HardwareIDs::rightJoystick};
        frc::Joystick leftJoystick{HardwareIDs::xBox};
        frc::XboxController xBox{HardwareIDs::xBox};
        frc::Solenoid solenoid{frc::PneumaticsModuleType::CTREPCM, 0};
};