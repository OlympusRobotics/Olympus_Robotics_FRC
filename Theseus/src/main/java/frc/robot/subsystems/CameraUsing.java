package frc.robot.subsystems;
// NOTE: Changes to camera names, positions, or vision config must be reflected in Theseus/README.md (Vision section).

import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

public class CameraUsing extends SubsystemBase {
    // Maximum ambiguity before rejecting a measurement entirely
    private static final double MAX_AMBIGUITY = 0.25;
    // Maximum distance (m) a vision pose can be from odometry before rejection
    private static final double MAX_POSE_JUMP = 15.0;
    // Field dimensions (2026 Rebuilt Andymark)
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;

    private final Transform3d robotToCamFL, robotToCamFR, robotToCamBL, robotToCamBR;
    private final PhotonCamera camFL, camFR, camBL, camBR;
    private final Camera cameraProcessor;
    private final CommandSwerveDrivetrain drivetrain;

    public CameraUsing(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        cameraProcessor = new Camera();

        camFL = new PhotonCamera("CameraFL");
        camFR = new PhotonCamera("CameraFR");
        camBL = new PhotonCamera("CameraBL");
        camBR = new PhotonCamera("CameraBR");

        robotToCamFL = new Transform3d(
                new Translation3d(-.15, .3048, .4826),
                new Rotation3d(0, 0, Math.PI / 4));

        robotToCamFR = new Transform3d(
                new Translation3d(-.15, -.3048, .4826),
                new Rotation3d(0, 0, -Math.PI / 4));

        robotToCamBL = new Transform3d(
                new Translation3d(-.3302, .2286, .152),
                new Rotation3d(0, 0, 3 * (Math.PI) / 4));

        robotToCamBR = new Transform3d(
                new Translation3d(-.3302, -.2286, .152),
                new Rotation3d(0, 0, -3 * (Math.PI) / 4));
    }

    private void processCamera() {
        int measurementCount = 0;
        Pose2d bestPose = null;
        double bestAmbiguity = MAX_AMBIGUITY;
        Pose2d currentPose = drivetrain.getState().Pose;

        for (var camPair : List.of(
                Map.entry(camBL, robotToCamBL), Map.entry(camBR, robotToCamBR),
                Map.entry(camFL, robotToCamFL), Map.entry(camFR, robotToCamFR))) {

            PhotonCamera cam = camPair.getKey();
            Transform3d robotToCam = camPair.getValue();

            // estimateFieldToRobotAprilTag expects camera-to-robot, so invert
            cameraProcessor.setCameraToRobot(robotToCam.inverse());
            Camera.CameraResult camData = cameraProcessor.process(cam);

            if (camData.robotPose() == null) {
                continue;
            }

            double ambiguity = camData.ambiguity();
            double distance = camData.distance();
            Pose2d visionPose = camData.robotPose().toPose2d();

            if (drivetrain.getState().Pose.getX() < 0 || drivetrain.getState().Pose.getX() > FIELD_LENGTH
                    || drivetrain.getState().Pose.getY() < 0 || drivetrain.getState().Pose.getY() > FIELD_WIDTH) {
                drivetrain.resetPose(visionPose);
            }

            // Reject garbage ambiguity
            if (ambiguity > MAX_AMBIGUITY) {
                continue;
            }

            // Reject poses outside field bounds
            if (visionPose.getX() < 0 || visionPose.getX() > FIELD_LENGTH
                    || visionPose.getY() < 0 || visionPose.getY() > FIELD_WIDTH) {
                continue;
            }

            // Reject poses that jump too far from current odometry
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) > MAX_POSE_JUMP) {
                continue;
            }

            // Scale std devs continuously by ambiguity and distance
            double xyStdDev = 0.01 + (ambiguity * 2.0) + (distance * 0.05);
            double thetaStdDev = Units.degreesToRadians(2 + ambiguity * 50 + distance * 10);

            drivetrain.addVisionMeasurement(
                    visionPose,
                    camData.pipelineResult().getTimestampSeconds(),
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

            measurementCount++;

            // Track the best pose for logging
            if (ambiguity < bestAmbiguity) {
                bestAmbiguity = ambiguity;
                bestPose = visionPose;
            }
            SmartDashboard.putNumber("bleh", visionPose.getX());
        }

        Logger.recordOutput("Vision/HasTarget", measurementCount > 0);
        Logger.recordOutput("Vision/MeasurementCount", measurementCount);
        if (bestPose != null) {
            Logger.recordOutput("Vision/EstimatedPose", bestPose);
            Logger.recordOutput("Vision/BestAmbiguity", bestAmbiguity);
        }
    }

    @Override
    public void periodic() {
        processCamera();
    }
}