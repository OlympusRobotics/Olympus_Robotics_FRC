import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Create a DCMotorSim for physics sim
        flDGearBox = DCMotor.krakenX60FOC(1)
        frDGearBox = DCMotor.krakenX60FOC(1)
        blDGearBox = DCMotor.krakenX60FOC(1)
        brDGearBox = DCMotor.krakenX60FOC(1)


        self.flDSIM = sim.DCMotorSim(LinearSystemId.DCMotorSystem(flDGearBox, 0.01, 6.75), flDGearBox)
        self.frDSIM = sim.DCMotorSim(LinearSystemId.DCMotorSystem(frDGearBox, 0.01, 6.75), frDGearBox)
        self.blDSIM = sim.DCMotorSim(LinearSystemId.DCMotorSystem(blDGearBox, 0.01, 6.75), blDGearBox)
        self.brDSIM = sim.DCMotorSim(LinearSystemId.DCMotorSystem(brDGearBox, 0.01, 6.75), brDGearBox)

        # Keep a reference to the motor sim state so we can update it
        self.flDSimState = robot.drivetrain.flSM.driveMotor.sim_state
        self.frDSimState = robot.drivetrain.frSM.driveMotor.sim_state
        self.blDSimState = robot.drivetrain.blSM.driveMotor.sim_state
        self.brDSimState = robot.drivetrain.brSM.driveMotor.sim_state

        self.gyroSim = robot.drivetrain.gyro.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # If the driver station is enabled, then feed enable for phoenix devices
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)


        self.flDSimState.set_supply_voltage(RobotController.getBatteryVoltage())
        self.frDSimState.set_supply_voltage(RobotController.getBatteryVoltage())
        self.blDSimState.set_supply_voltage(RobotController.getBatteryVoltage())
        self.brDSimState.set_supply_voltage(RobotController.getBatteryVoltage())

        self.flDSIM.setInputVoltage(self.flDSimState.motor_voltage)
        self.frDSIM.setInputVoltage(self.frDSimState.motor_voltage)
        self.blDSIM.setInputVoltage(self.blDSimState.motor_voltage)
        self.brDSIM.setInputVoltage(self.brDSimState.motor_voltage)

        self.flDSIM.update(tm_diff)
        self.frDSIM.update(tm_diff)
        self.blDSIM.update(tm_diff)
        self.brDSIM.update(tm_diff)

        self.flDSimState.set_raw_rotor_position(radiansToRotations(self.flDSIM.getAngularPosition()))
        self.flDSimState.set_rotor_velocity(radiansToRotations(self.flDSIM.getAngularVelocity()))

        self.frDSimState.set_raw_rotor_position(radiansToRotations(self.frDSIM.getAngularPosition()))
        self.frDSimState.set_rotor_velocity(radiansToRotations(self.frDSIM.getAngularVelocity()))

        self.blDSimState.set_raw_rotor_position(radiansToRotations(self.blDSIM.getAngularPosition()))
        self.blDSimState.set_rotor_velocity(radiansToRotations(self.blDSIM.getAngularVelocity()))

        self.brDSimState.set_raw_rotor_position(radiansToRotations(self.brDSIM.getAngularPosition()))
        self.brDSimState.set_rotor_velocity(radiansToRotations(self.brDSIM.getAngularVelocity()))
