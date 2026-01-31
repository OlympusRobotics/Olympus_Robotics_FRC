package frc.robot.subsystems;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Camera.datatatatata;
import frc.robot.subsystems.Drivetrain;

public class CameraUsing {
    private Transform3d robotToCamFL;
    private Transform3d robotToCamFR;
    private Transform3d robotToCamBL;
    private Transform3d robotToCamBR;
    private PhotonCamera camFLL;
    private PhotonCamera camFRR;
    private PhotonCamera camBLL;
    private PhotonCamera camBRR;
    private PhotonPoseEstimator estimatorerorer;
    private Camera camerafile;
    private double robotx;
    private double roboty;
    private double robotz;
    private Rotation3d robotRotation;
    private Pose3d robotPose3d;
    private Pose2d robotPose2d;
    
    
    public void CameraStuff() {
        camFLL = new PhotonCamera("CameraFL");
        robotToCamFL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, Math.PI/4));

        camFRR = new PhotonCamera("CameraFR");
        robotToCamFR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, -Math.PI/4));

        camBLL = new PhotonCamera("CameraBL");
        robotToCamBL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, 3*(Math.PI)/4));

        camBRR = new PhotonCamera("CameraBR");
        robotToCamBR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, -3*(Math.PI)/4));

        var camFL = camerafile.cameraProcessing(camFLL);
        var camFR = camerafile.cameraProcessing(camFRR);
        var camBL = camerafile.cameraProcessing(camBLL);
        var camBR = camerafile.cameraProcessing(camBRR);

        robotx = ((camFL.robotpose().getX() + camFR.robotpose().getX() + camBR.robotpose().getX() + camBL.robotpose().getX())/4);
        roboty = ((camFL.robotpose().getY() + camFR.robotpose().getY() + camBR.robotpose().getY() + camBL.robotpose().getY())/4);
        robotz = ((camFL.robotpose().getZ() + camFR.robotpose().getZ() + camBR.robotpose().getZ() + camBL.robotpose().getZ())/4);
        robotRotation = ((camBL.robotpose().getRotation().plus(camBR.robotpose().getRotation()).plus( camFR.robotpose().getRotation()).plus(camFL.robotpose().getRotation())).div(4));

        robotPose2d = new Pose2d(robotx, roboty, Drivetrain.getRobotRotation());
    }
}
