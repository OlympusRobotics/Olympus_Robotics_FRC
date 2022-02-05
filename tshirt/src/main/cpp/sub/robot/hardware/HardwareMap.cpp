#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/motorcontrol/Victor.h>
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        frc::Talon talon1{4};
        frc::Talon talon2{5};
        frc::Victor leftMotor1{HardwareIDs::leftMotor1};
        frc::Victor leftMotor2{HardwareIDs::leftMotor2};
        frc::Victor rightMotor1{HardwareIDs::rightMotor1};
        frc::Victor rightMotor2{HardwareIDs::rightMotor2};
        frc::Joystick rightJoystick{HardwareIDs::rightJoystick};
        frc::Joystick leftJoystick{HardwareIDs::leftJoystick};
        frc::XboxController xBox{HardwareIDs::xBox};
};