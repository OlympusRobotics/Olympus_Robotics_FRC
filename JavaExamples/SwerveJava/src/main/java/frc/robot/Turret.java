package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Turret subsystem controls the rotational aiming of the shooter assembly.
 * The turret can rotate within mechanical limits (typically +/- 120 degrees).
 * 
 * This turret uses:
 * - TalonFX motor controller for rotation
 * - Digital limit switches for mechanical limits
 * - Position control for precise aiming
 */
public class Turret extends SubsystemBase {
    private final TalonFX m_turretMotor;
    
    // Control requests
    private final PositionVoltage m_positionControl = new PositionVoltage(0);
    private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);
    
    // Constants - adjust these for your robot
    private static final int TURRET_MOTOR_ID = 12;
    
    // Turret mechanical limits in degrees
    private static final double MAX_TURRET_ANGLE = 120.0;
    private static final double MIN_TURRET_ANGLE = -120.0;
    
    // Soft limits (slightly inside mechanical limits for safety)
    private static final double SOFT_MAX_TURRET_ANGLE = 115.0;
    private static final double SOFT_MIN_TURRET_ANGLE = -115.0;
    
    // Gear ratio (motor rotations : turret rotations)
    private static final double GEAR_RATIO = 60.0; // Adjust for your mechanism
    
    // Tolerance for on-target checking (degrees)
    private static final double ON_TARGET_TOLERANCE = 2.0;
    private static final double SAFE_POSITION_TOLERANCE = 5.0;
    
    // PID values - tune these for your robot
    private static final double kP = 24.0;
    private static final double kI = 0.0;
    private static final double kD = 0.5;
    private static final double kS = 0.1; // Feedforward static gain
    private static final double kV = 0.12; // Feedforward velocity gain

    public Turret() {
        m_turretMotor = new TalonFX(TURRET_MOTOR_ID);
        
        // Configure the motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = false;
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID configuration
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        config.Slot0 = slot0;
        
        // Software limit switches
        SoftwareLimitSwitchConfigs limitConfig = new SoftwareLimitSwitchConfigs();
        limitConfig.ForwardSoftLimitEnable = true;
        limitConfig.ReverseSoftLimitEnable = true;
        limitConfig.ForwardSoftLimitThreshold = degreesToRotations(SOFT_MAX_TURRET_ANGLE);
        limitConfig.ReverseSoftLimitThreshold = degreesToRotations(SOFT_MIN_TURRET_ANGLE);
        config.SoftwareLimitSwitch = limitConfig;
        
        // Apply configuration
        m_turretMotor.getConfigurator().apply(config);
        
        // Set encoder position to 0 (assumes turret starts at center)
        m_turretMotor.setPosition(0);
    }

    /**
     * Set the desired angle of the turret using position control
     * 
     * @param angle Desired angle as a Rotation2d object
     */
    public void setDesiredAngle(Rotation2d angle) {
        double rotations = degreesToRotations(angle.getDegrees());
        m_turretMotor.setControl(m_positionControl.withPosition(rotations));
    }

    /**
     * Manually control the turret with open-loop control
     * 
     * @param speed Motor speed from -1.0 to 1.0
     */
    public void setOpenLoop(double speed) {
        m_turretMotor.setControl(m_dutyCycleControl.withOutput(speed));
    }

    /**
     * Reset the turret encoder to a known position
     * 
     * @param actualRotation The actual rotation of the turret
     */
    public void reset(Rotation2d actualRotation) {
        m_turretMotor.setPosition(degreesToRotations(actualRotation.getDegrees()));
    }

    /**
     * Get the current angle of the turret
     * 
     * @return Current turret angle as Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(rotationsToDegrees(m_turretMotor.getPosition().getValueAsDouble()));
    }

    /**
     * Get the current setpoint in degrees
     * 
     * @return Setpoint in degrees
     */
    public double getSetpointDegrees() {
        return rotationsToDegrees(m_turretMotor.getClosedLoopReference().getValueAsDouble());
    }

    /**
     * Get the error between current position and setpoint
     * 
     * @return Error in degrees
     */
    private double getErrorDegrees() {
        return getAngle().getDegrees() - getSetpointDegrees();
    }

    /**
     * Check if the turret is on target
     * 
     * @return true if within tolerance of setpoint
     */
    public boolean isOnTarget() {
        return Math.abs(getErrorDegrees()) < ON_TARGET_TOLERANCE;
    }

    /**
     * Check if the turret is in a safe position (near center)
     * 
     * @return true if turret is near center position
     */
    public boolean isSafe() {
        return Math.abs(getAngle().getDegrees()) < SAFE_POSITION_TOLERANCE;
    }

    /**
     * Go to the center/home position
     */
    public void goToHome() {
        setDesiredAngle(new Rotation2d());
    }

    /**
     * Stop the turret motor
     */
    public void stop() {
        setOpenLoop(0);
    }

    /**
     * Convert degrees to motor rotations
     */
    private double degreesToRotations(double degrees) {
        return (degrees / 360.0) * GEAR_RATIO;
    }

    /**
     * Convert motor rotations to degrees
     */
    private double rotationsToDegrees(double rotations) {
        return (rotations / GEAR_RATIO) * 360.0;
    }

    @Override
    public void periodic() {
        // Update SmartDashboard
        SmartDashboard.putNumber("Turret/Angle (deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("Turret/Setpoint (deg)", getSetpointDegrees());
        SmartDashboard.putNumber("Turret/Error (deg)", getErrorDegrees());
        SmartDashboard.putBoolean("Turret/On Target", isOnTarget());
        SmartDashboard.putBoolean("Turret/Is Safe", isSafe());
        SmartDashboard.putNumber("Turret/Motor Current", m_turretMotor.getStatorCurrent().getValueAsDouble());
    }
}