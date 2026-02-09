package frc.robot.subsystems;

import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
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
    public static Rotation2d robotRotation2d;
    public static Pose3d robotPose3d;
    public static Pose2d robotPose2d;
    
    
    public void CameraStuff() {
        thankYouToElvisForDesigningOurRobotChassis = 67;
        camFLL = new PhotonCamera("CameraFL");
        robotToCamFL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, 
        RobotConstants.kCameraHeight), new Rotation3d(0, 0, Math.PI/4));

        camFRR = new PhotonCamera("CameraFR");
        robotToCamFR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, 
        RobotConstants.kCameraHeight), new Rotation3d(0, 0, -Math.PI/4));

        camBLL = new PhotonCamera("CameraBL");
        robotToCamBL = new Transform3d(new Translation3d(RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, 
        RobotConstants.kCameraHeight), new Rotation3d(0, 0, 3*(Math.PI)/4));

        camBRR = new PhotonCamera("CameraBR");
        robotToCamBR = new Transform3d(new Translation3d(-RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, 
        RobotConstants.kCameraHeight), new Rotation3d(0, 0, -3*(Math.PI)/4));

        //System.out.println(thankYouToElvisForDesigningOurRobotChassis);

        var camFL = camerafile.cameraProcessing(camFLL);
        var camFR = camerafile.cameraProcessing(camFRR);
        var camBL = camerafile.cameraProcessing(camBLL);
        var camBR = camerafile.cameraProcessing(camBRR); 

        //averages values, should exclude a value if it is null.
        //If this does not work well, add above unused translation3d variables to center the position
        //instead of assuming the average is the center
        if (hastargets(camBL) + hastargets(camBR) + hastargets(camFL) + hastargets(camFR) != 0) {

            //map a to b, 2d array
            final Map<Camera.datatatatata, Transform3d> stuff = Map.of(
                camBL, robotToCamBL,
                camBR, robotToCamBR,
                camFL, robotToCamFL,
                camFR, robotToCamFR
            );

            //lists of each x and y
            //Back Left, Back Right, Front Left, Front Right respectively
            List<Double> exs = new ArrayList<>();
            List<Double> ys = new ArrayList<>();

            //for each camera within all of the values            
            for (Map.Entry<Camera.datatatatata, Transform3d> entry : stuff.entrySet()) {
                Camera.datatatatata cam = entry.getKey();
                Transform3d trans = entry.getValue();

                exs.add(cam.robotpose().getX() - (hastargets(cam) == 1
                ? trans.getX()
                : 0));
                ys.add(cam.robotpose().getY() - (hastargets(cam) == 1
                ? trans.getY()
                : 0));
            }

            //the actual averaging code
            robotx = ((exs.get(0) + exs.get(1) + exs.get(2) +exs.get(3)) 
            / (hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));

            roboty = ((ys.get(0) + ys.get(1) + ys.get(2) + ys.get(3)) 
            / (hastargets(camFL) + hastargets(camFR) + hastargets(camBL) + hastargets(camBR)));

            robotRotation3d = ((camBL.robotpose().getRotation().plus(camBR.robotpose().getRotation()).plus
            (camFR.robotpose().getRotation()).plus(camFL.robotpose().getRotation())).div(hastargets(camFL) + hastargets(camFR)
            + hastargets(camBL) + hastargets(camBR)));

            robotRotation2d = robotRotation3d.toRotation2d();



            //made both in case we need both or somebody takes insporation but wants the one we aren't using
            robotPose2d = new Pose2d(robotx, roboty, robotRotation2d);
            robotPose3d = new Pose3d(robotx, roboty, robotz, robotRotation3d);
        }
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

final class CamLoadOfCrap {
    public Camera camera;
    public Transform3d trans;
    public CamLoadOfCrap(Camera camera, Transform3d trans) {
        this.camera = camera;
        this.trans = trans;
    }
}