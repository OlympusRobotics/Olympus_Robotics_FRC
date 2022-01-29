#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "HardwareIDs.cpp"

class HardwareMap {
    public:
        TalonSRX intakeMotor = {HardWareIDs::intakeMotor};
        TalonSRX outputMotor1 = {HardWareIDs::outputMotor1};
        TalonSRX outputMotor2 = {HardWareIDs::outputMotor2};
        TalonSRX beltMotor = {HardWareIDs::beltMotor};
        VictorSPX frontLeftMotor = {HardWareIDs::frontLeftMotor};
        VictorSPX frontRightMotor = {HardWareIDs::frontRightMotor};
        VictorSPX rearLeftMotor = {HardWareIDs::rearLeftMotor};
        VictorSPX rearRightMotor = {HardWareIDs::rearRightMotor};
};