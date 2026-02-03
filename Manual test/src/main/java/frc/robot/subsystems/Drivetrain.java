package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Drivetrain extends SubsystemBase{
    RobotConstants Constants = new RobotConstants();
    private final Swerve flSM = new Swerve(
        RobotConstants.kFrontLeftDriveID, 
        RobotConstants.kFrontLeftRotationID, 
        RobotConstants.kFrontLeftEncoderID, 
        RobotConstants.kFrontLeftEncoderOffset
        );

    private final Swerve frSM = new Swerve(
        RobotConstants.kFrontRightDriveID, 
        RobotConstants.kFrontRightRotationID, 
        RobotConstants.kFrontRightEncoderID, 
        RobotConstants.kFrontRightEncoderOffset
        );

    private final Swerve blSM = new Swerve(
        RobotConstants.kBackLeftDriveID, 
        RobotConstants.kBackLeftRotationID, 
        RobotConstants.kBackLeftEncoderID, 
        RobotConstants.kBackLeftEncoderOffset
        );

    private final Swerve brSM = new Swerve(
        RobotConstants.kBackRightDriveID, 
        RobotConstants.kBackRightRotationID,
        RobotConstants.kBackRightEncoderID,
        RobotConstants.kBackRightEncoderOffset
        );

    static public final Pigeon2 Gyro = new Pigeon2(RobotConstants.kGyroID);
    private final SwerveDriveOdometry m_odometry;

    Translation2d m_frontLeftLocation = new Translation2d(0.5334, 0.5334);
    Translation2d m_frontRightLocation = new Translation2d(0.5334, -0.5334);
    Translation2d m_backLeftLocation = new Translation2d(-0.5334, 0.5334);
    Translation2d m_backRightLocation = new Translation2d(-0.5334, -0.5334);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    public Drivetrain() {
        Gyro.setYaw(0);
        
        RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception except) {
      // Handle exception as needed
      except.printStackTrace();
    }

        flSM.RotPID(RobotConstants.kFrontLeftP, RobotConstants.kFrontLeftI, RobotConstants.kFrontLeftD);
        frSM.RotPID(RobotConstants.kFrontRightP, RobotConstants.kFrontRightI, RobotConstants.kFrontRightD);
        blSM.RotPID(RobotConstants.kBackLeftP, RobotConstants.kBackLeftI, RobotConstants.kBackLeftD);
        brSM.RotPID(RobotConstants.kBackRightP, RobotConstants.kBackRightI, RobotConstants.kBackRightD);

        m_odometry = new SwerveDriveOdometry(
        m_kinematics, Gyro.getRotation2d(),
        new SwerveModulePosition[] {
            flSM.getPosition(),
            frSM.getPosition(),
            blSM.getPosition(),
            brSM.getPosition()
        }, new Pose2d(0, 0, new Rotation2d())
        );
        

    //     AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> drive2(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }

    public static boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        else {
            //assume blue
            return false;
        }
    }

    public void resetPose(Pose2d newpose){
        m_odometry.resetPosition(Gyro.getRotation2d(), new SwerveModulePosition[] {
            flSM.getPosition(),
            frSM.getPosition(),
            blSM.getPosition(),
            brSM.getPosition()
        }, newpose);
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }
    public void fieldOrientedDrive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed, rot, true);
    }
    public void robotOrientedDrive(double xSpeed, double ySpeed, double rot) {
        drive(xSpeed, ySpeed, rot, false);
    }

    public Rotation2d getRobotRotation(){
        return Gyro.getRotation2d();
    }

    public ChassisSpeeds getChassisSpeeds(){
        return speeds;
    }

    public void updateOdometry(){
        Pose2d updated_odometry = m_odometry.update(
            Gyro.getRotation2d(),
            new SwerveModulePosition[] {
                flSM.getPosition(),
                frSM.getPosition(),
                blSM.getPosition(),
                brSM.getPosition()
            }
        );
        if (updated_odometry != CameraUsing.robotPose2d && CameraUsing.robotPose2d != new Pose2d()) {
            m_odometry.resetPose(CameraUsing.robotPose2d);
        }
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.kMaxModuleSpeed);
        flSM.setDesiredState(moduleStates[0]);
        frSM.setDesiredState(moduleStates[1]);
        blSM.setDesiredState(moduleStates[2]);
        brSM.setDesiredState(moduleStates[3]);
    }
    public void drive2(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConstants.kMaxModuleSpeed);
        flSM.setDesiredState(moduleStates[0]);
        frSM.setDesiredState(moduleStates[1]);
        blSM.setDesiredState(moduleStates[2]);
        brSM.setDesiredState(moduleStates[3]);
    }

    public void stopmotors(){
        flSM.stopmotors();
        frSM.stopmotors();
        blSM.stopmotors();
        brSM.stopmotors();
    }
}