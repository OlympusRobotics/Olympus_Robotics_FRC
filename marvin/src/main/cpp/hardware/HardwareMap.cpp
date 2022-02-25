#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        TalonSRX frontLeftMotor = {HardwareIDs::fLM};
        TalonSRX backLeftMotor = {HardwareIDs::bLM};
        TalonSRX frontRightMotor = {HardwareIDs::fRM};
        TalonSRX backRightMotor = {HardwareIDs::bRM};
        frc::Joystick leftJoystick{HardwareIDs::lJ};
        frc::Joystick rightJoystick{HardwareIDs::rJ};
        frc::XboxController xBox{HardwareIDs::xBox};
};