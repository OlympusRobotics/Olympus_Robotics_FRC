#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        TalonSRX intakeMotor = {HardwareIDs::intakeMotor};
        TalonSRX outputMotor1 = {HardwareIDs::outputMotor1};
        TalonSRX outputMotor2 = {HardwareIDs::outputMotor2};
        TalonSRX beltMotor = {HardwareIDs::beltMotor};
        VictorSPX frontLeftMotor = {HardwareIDs::frontLeftMotor};
        VictorSPX frontRightMotor = {HardwareIDs::frontRightMotor};
        VictorSPX rearLeftMotor = {HardwareIDs::rearLeftMotor};
        VictorSPX rearRightMotor = {HardwareIDs::rearRightMotor};
        frc::Joystick rightJoystick{HardwareIDs::rightJoystick};
        frc::Joystick leftJoystick{HardwareIDs::leftJoystick};
        frc::XboxController xBox{HardwareIDs::xBox};
        frc::Solenoid solenoid{frc::PneumaticsModuleType::CTREPCM, 0};
};