package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Intake subsystem controls the intake roller mechanism.
 * It can intake game pieces or eject them back to the field.
 * 
 * The intake has two motors:
 * - Outer roller motor (main intake)
 * - Fixed roller motor (secondary roller)
 * 
 * It also has a pneumatic deploy mechanism and a sensor to detect
 * if a game piece is in the intake.
 */
public class Intake extends SubsystemBase {
    // Motor Controllers
    private final SparkMax m_intakeMotor;
    private final SparkMax m_fixedMotor;
    
    // Pneumatics
    private final Solenoid m_deploySolenoid;
    
    // Sensors
    private final AnalogInput m_gamePieceSensor;
    
    // Constants - adjust these for your robot
    private static final int INTAKE_MOTOR_ID = 10;
    private static final int FIXED_MOTOR_ID = 11;
    private static final int DEPLOY_SOLENOID_CHANNEL = 0;
    private static final int GAME_PIECE_SENSOR_CHANNEL = 4;
    private static final double GAME_PIECE_VOLTAGE_THRESHOLD = 1.5;
    private static final double INTAKE_SPEED = 0.8;
    private static final double FIXED_ROLLER_SPEED = 0.6;

    public Intake() {
        // Initialize intake motor
        m_intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize fixed roller motor
        m_fixedMotor = new SparkMax(FIXED_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig fixedConfig = new SparkMaxConfig();
        fixedConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        m_fixedMotor.configure(fixedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize pneumatics
        m_deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, DEPLOY_SOLENOID_CHANNEL);
        
        // Initialize sensor
        m_gamePieceSensor = new AnalogInput(GAME_PIECE_SENSOR_CHANNEL);
    }

    /**
     * Set the intake roller motors
     * Positive power intakes game pieces, negative ejects them
     * 
     * @param outerPower Power for the main intake motor (-1.0 to 1.0)
     * @param fixedPower Power for the fixed roller motor (-1.0 to 1.0)
     */
    public void setIntakeRoller(double outerPower, double fixedPower) {
        // Reduce intake power if we already have a game piece
        if (hasGamePiece()) {
            m_intakeMotor.set(Math.max(-outerPower, 0)); // Only allow holding/ejecting
        } else {
            m_intakeMotor.set(-outerPower);
        }
        m_fixedMotor.set(-fixedPower);
    }

    /**
     * Run intake at default speed
     */
    public void intake() {
        setIntakeRoller(INTAKE_SPEED, FIXED_ROLLER_SPEED);
    }

    /**
     * Eject game piece
     */
    public void eject() {
        setIntakeRoller(-INTAKE_SPEED, -FIXED_ROLLER_SPEED);
    }

    /**
     * Deploy or retract the intake
     * 
     * @param deploy true to deploy, false to retract
     */
    public void setDeploy(boolean deploy) {
        m_deploySolenoid.set(deploy);
    }

    /**
     * Check if intake has a game piece
     * 
     * @return true if game piece is detected
     */
    public boolean hasGamePiece() {
        return m_gamePieceSensor.getAverageVoltage() > GAME_PIECE_VOLTAGE_THRESHOLD;
    }

    /**
     * Stop all intake motors
     */
    public void stop() {
        setIntakeRoller(0, 0);
        setDeploy(false);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with intake status
        double voltage = m_gamePieceSensor.getAverageVoltage();
        SmartDashboard.putBoolean("Intake/Has Game Piece", hasGamePiece());
        SmartDashboard.putNumber("Intake/Sensor Voltage", voltage);
        SmartDashboard.putNumber("Intake/Motor Current", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Deployed", m_deploySolenoid.get());
    }
}