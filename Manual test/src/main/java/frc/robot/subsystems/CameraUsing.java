package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class CameraUsing extends SubsystemBase {
    private Transform3d robotToCamFL, robotToCamFR, robotToCamBL, robotToCamBR;
    private PhotonCamera camFL, camFR, camBL, camBR;
    private Camera camerafile;
    private double thankYouToElvisForDesigningOurRobotChassis;
    public static Rotation2d robotRotation2d;
    public static Pose2d robotPose2d;
    private Drivetrain drivetrain;

        public CameraUsing(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
            camerafile = new Camera();
            //thankYouToElvisForDesigningOurRobotChassis = 67;
            camerafile = new Camera();
            camFL = new PhotonCamera("CameraFL");
            camFR = new PhotonCamera("CameraFR");
            camBL = new PhotonCamera("CameraBL");
            camBR = new PhotonCamera("CameraBR");

            robotToCamFL = new Transform3d(
                    new Translation3d(RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight),
                    new Rotation3d(0, 0, Math.PI/4));

            robotToCamFR = new Transform3d(
                    new Translation3d(-RobotConstants.kTrackWidth/2, RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight),
                    new Rotation3d(0, 0, -Math.PI/4));

            robotToCamBL = new Transform3d(
                    new Translation3d(RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight),
                    new Rotation3d(0, 0, 3*(Math.PI)/4));

            robotToCamBR = new Transform3d(
                    new Translation3d(-RobotConstants.kTrackWidth/2, -RobotConstants.kRobotlength/2, RobotConstants.kCameraHeight),
                    new Rotation3d(0, 0, -3*(Math.PI)/4));
        }
        //System.out.println(thankYouToElvisForDesigningOurRobotChassis);
    private void processCamera() {
        double lowestAmbiguity = .3;

        for (var camPair : List.of(
                Map.entry(camBL, robotToCamBL), Map.entry(camBR, robotToCamBR),
                Map.entry(camFL, robotToCamFL), Map.entry(camFR, robotToCamFR))) {

            PhotonCamera cam = camPair.getKey();
            Transform3d transform = camPair.getValue();

            camerafile.setCameraToRobot(transform);
            var camData = camerafile.cameraProcessing(cam);

            if (camData.robotpose() != null && camData.result().hasTargets()) {
                drivetrain.addVisionMeasurement(camData.robotpose().toPose2d(), camData.result().getTimestampSeconds());

                double ambiguity = camData.result().getBestTarget().getPoseAmbiguity();
                if (ambiguity < lowestAmbiguity) {
                    drivetrain.addVisionMeasurement(
                        camData.robotpose().toPose2d(),
                        camData.result().getTimestampSeconds()
                    );
                }
            }
        }
    }
    @Override
    public void periodic() {
        processCamera();
    }
}