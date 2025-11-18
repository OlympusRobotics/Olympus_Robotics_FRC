package frc.robot.Subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

    //Instantiate Motor Objects
    private final SparkMax m_leftMotor1;
    private final SparkMax m_leftMotor2;
    private final SparkMax m_rightMotor1;
    private final SparkMax m_rightMotor2;

    //Configure Kinematics
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    
    public Drivetrain() {
        // Initialize motors
        m_leftMotor1 = new SparkMax(Constants.leftMotor1_ID, MotorType.kBrushless);
        m_leftMotor2 = new SparkMax(Constants.leftMotor2_ID, MotorType.kBrushless);

        m_rightMotor1 = new SparkMax(Constants.rightMotor1_ID, MotorType.kBrushless);
        m_rightMotor2 = new SparkMax(Constants.rightMotor2_ID, MotorType.kBrushless);

        // 
        SparkMaxConfig m_masterConfig = new SparkMaxConfig();
        SparkMaxConfig m_followerConfig1 = new SparkMaxConfig();
        SparkMaxConfig m_followerConfig2 = new SparkMaxConfig();
        
        //edit config for both master and follower motors
        m_masterConfig
            .smartCurrentLimit(Constants.driveCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        m_followerConfig1
            .smartCurrentLimit(Constants.driveCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .follow(Constants.leftMotor1_ID);

        m_followerConfig2
            .smartCurrentLimit(Constants.driveCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .follow(Constants.rightMotor1_ID);
            
        //Save configs to motors
        m_leftMotor1.configure(m_masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftMotor2.configure(m_followerConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor1.configure(m_masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor2.configure(m_followerConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void drive(double x, double omega){
        /* Drive Function */

        //Convert controller values into wheel speeds
        DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(x, 0, omega));

        //sets wheel speeds to the motor
        m_leftMotor1.set(speeds.leftMetersPerSecond);
        m_rightMotor1.set(speeds.rightMetersPerSecond);

    }

    public void stopDrive(){
        m_leftMotor1.stopMotor();
        m_rightMotor1.stopMotor();
    }

}
