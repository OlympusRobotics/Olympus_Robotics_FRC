package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Camera.datatatatata;

public class CameraUsing {
    private Transform3d robotToCamFL;
    private Transform3d robotToCamFR;
    private Transform3d robotToCamBL;
    private Transform3d robotToCamBR;
    private PhotonCamera camFLL;
    private PhotonCamera camFRR;
    private PhotonCamera camBLL;
    private PhotonCamera camBRR;
    private Camera camerafile;
    private double robotx;
    private double roboty;
    private double robotz;
    private double thankYouToElvisForDesigningOurRobotChassis;
    private Rotation3d robotRotation3d;
    private Rotation2d robotRotation2d;
    private Pose3d robotPose3d;
    private Pose2d robotPose2d;
    
    
    public void CameraStuff() {
        thankYouToElvisForDesigningOurRobotChassis = 67;
        camFLL = new PhotonCamera("CameraFL");
        robotToCamFL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, Math.PI/4));

        camFRR = new PhotonCamera("CameraFR");
        robotToCamFR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, -Math.PI/4));

        camBLL = new PhotonCamera("CameraBL");
        robotToCamBL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, 3*(Math.PI)/4));

        camBRR = new PhotonCamera("CameraBR");
        robotToCamBR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight), new Rotation3d(0, 0, -3*(Math.PI)/4));

        //System.out.println(thankYouToElvisForDesigningOurRobotChassis);

        var camFL = camerafile.cameraProcessing(camFLL);
        var camFR = camerafile.cameraProcessing(camFRR);
        var camBL = camerafile.cameraProcessing(camBLL);
        var camBR = camerafile.cameraProcessing(camBRR); 

        //averages values, should exclude a value if it is null
        robotx = ((camFL.robotpose().getX() + camFR.robotpose().getX() + camBR.robotpose().getX() + camBL.robotpose().getX())/(hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));
        roboty = ((camFL.robotpose().getY() + camFR.robotpose().getY() + camBR.robotpose().getY() + camBL.robotpose().getY())/(hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));
        robotz = ((camFL.robotpose().getZ() + camFR.robotpose().getZ() + camBR.robotpose().getZ() + camBL.robotpose().getZ())/(hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));

        //same thing as above without converting to doubles
        robotRotation3d = ((camBL.robotpose().getRotation().plus(camBR.robotpose().getRotation()).plus( camFR.robotpose().getRotation()).plus(camFL.robotpose().getRotation())).div(hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));
        robotRotation2d = robotRotation3d.toRotation2d();

        //made both in case we need both or somebody takes insporation but wants the one we aren't using
        robotPose2d = new Pose2d(robotx, roboty, robotRotation2d);
        robotPose3d = new Pose3d(robotx, roboty, robotz, robotRotation3d);
    }
    public int hastargets(datatatatata camera) {
        if (camera.result().hasTargets()) {
            return 1;
        }
        else {
            return 0;
        }
        
    }
}
