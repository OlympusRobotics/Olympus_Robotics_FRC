#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"

class HardwareMap {
    public:
        TalonSRX intakeMotor = {4};
        TalonSRX outputMotor1 = {5};
        TalonSRX outputMotor2 = {6};
        TalonSRX beltMotor = {7};
        VictorSPX frontLeftMotor = {0};
        VictorSPX frontRightMotor = {1};
        VictorSPX rearLeftMotor = {2};
        VictorSPX rearRightMotor = {3};
};