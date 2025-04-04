package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase{
    //SwerveModule Initialization
    private final SwerveModule m_flSM = new SwerveModule(
        RobotConstants.kFrontLeftDriveID, 
        RobotConstants.kFrontLeftRotationID, 
        RobotConstants.kFrontLeftEncoderID, 
        RobotConstants.kFrontLeftEncoderOffset
        );

    private final SwerveModule m_frSM = new SwerveModule(
        RobotConstants.kFrontRightDriveID, 
        RobotConstants.kFrontRightRotationID, 
        RobotConstants.kFrontRightEncoderID, 
        RobotConstants.kFrontRightEncoderOffset
        );

    private final SwerveModule m_blSM = new SwerveModule(
        RobotConstants.kBackLeftDriveID, 
        RobotConstants.kBackLeftRotationID, 
        RobotConstants.kBackLeftEncoderID, 
        RobotConstants.kBackLeftEncoderOffset
        );

    private final SwerveModule m_brSM = new SwerveModule(
        RobotConstants.kBackRightDriveID, 
        RobotConstants.kBackRightRotationID,
        RobotConstants.kBackRightEncoderID,
        RobotConstants.kBackRightEncoderOffset
        );

    //Gyro Initialization
    private final Pigeon2 m_gyro = new Pigeon2(RobotConstants.kGyroID);


    //Module Locations
    Translation2d m_frontLeftLocation = new Translation2d(RobotConstants.kDimension, RobotConstants.kDimension);
    Translation2d m_frontRightLocation = new Translation2d(RobotConstants.kDimension, -RobotConstants.kDimension);
    Translation2d m_backLeftLocation = new Translation2d(-RobotConstants.kDimension, RobotConstants.kDimension);
    Translation2d m_backRightLocation = new Translation2d(-RobotConstants.kDimension, -RobotConstants.kDimension);

    //Kinematics Initialization
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation, 
        m_backRightLocation
    );

    ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);


    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics, 
        m_gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            m_flSM.getPosition(),
            m_frSM.getPosition(),
            m_blSM.getPosition(),
            m_brSM.getPosition()
        }
    );

    public Drivetrain(){
        m_gyro.setYaw(0.0);
    
        m_flSM.setPIDValues(RobotConstants.kFrontLeftP, 0, RobotConstants.kFrontLeftD);
        m_frSM.setPIDValues(RobotConstants.kFrontRightP, 0, RobotConstants.kFrontRightD);
        m_blSM.setPIDValues(RobotConstants.kBackLeftP, 0, RobotConstants.kBackLeftD);
        m_brSM.setPIDValues(RobotConstants.kBackRightP, 0, RobotConstants.kBackRightD);
    }

    public void resetPose(){
        m_gyro.setYaw(0.0);
        m_odometry = new SwerveDriveOdometry(m_kinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
            m_flSM.getPosition(),
            m_frSM.getPosition(),
            m_blSM.getPosition(),
            m_brSM.getPosition()
        }
        );
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public ChassisSpeeds getChassisSpeeds(){
        return m_chassisSpeeds;
    }

    public Rotation2d getRobotRotation(){
        return m_gyro.getRotation2d();
    }

    public void updateOdometry(){
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_flSM.getPosition(),
                m_frSM.getPosition(),
                m_blSM.getPosition(),
                m_brSM.getPosition()
            }
        );
    }

    public void stopDrive(){
        m_flSM.stopDrive();
        m_frSM.stopDrive();
        m_blSM.stopDrive();
        m_brSM.stopDrive();
    }

    public void drive(ChassisSpeeds speeds){
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

        m_flSM.setState(swerveModuleStates[0]);
        m_frSM.setState(swerveModuleStates[1]);
        m_blSM.setState(swerveModuleStates[2]);
        m_brSM.setState(swerveModuleStates[3]);
    }

}
