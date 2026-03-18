package frc.robot.subsystems;
// NOTE: Changes to camera names, positions, or vision config must be reflected in Theseus/README.md (Vision section).

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

public class CameraUsing extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean hasSeededPose = false;
    private static final int SEED_READING_COUNT = 5;
    private int goodReadingCount = 0;
    private static final double CAM_YAW_OFFSET = 0.256;

    private record CameraEntry(PhotonCamera camera, Camera estimator) {}
    private final List<CameraEntry> cameras;

    public CameraUsing(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Physical camera positions relative to robot center (robot-to-camera transforms)
        var robotToCamFL = new Transform3d(
                new Translation3d(-.15, .3048, .4826),
                new Rotation3d(0, 0, Math.PI/4 + CAM_YAW_OFFSET));
        var robotToCamFR = new Transform3d(
                new Translation3d(-.15, -.3048, .4826),
                new Rotation3d(0, 0, -Math.PI/4 + CAM_YAW_OFFSET));
        var robotToCamBL = new Transform3d(
                new Translation3d(-.3302, .2286, .152),
                new Rotation3d(0, 0, 3*(Math.PI)/4 + CAM_YAW_OFFSET));
        var robotToCamBR = new Transform3d(
                new Translation3d(-.3302, -.2286, .152),
                new Rotation3d(0, 0, -3*(Math.PI)/4 + CAM_YAW_OFFSET));

        var camFL = new PhotonCamera("CameraFL");
        var camFR = new PhotonCamera("CameraFR");
        var camBL = new PhotonCamera("CameraBL");
        var camBR = new PhotonCamera("CameraBR");

        cameras = List.of(
            new CameraEntry(camFL, new Camera(camFL, robotToCamFL)),
            new CameraEntry(camFR, new Camera(camFR, robotToCamFR)),
            new CameraEntry(camBL, new Camera(camBL, robotToCamBL)),
            new CameraEntry(camBR, new Camera(camBR, robotToCamBR))
        );
    }

    private void processCamera() {
        int measurementCount = 0;
        double bestAmbiguity = 1.0;
        Pose2d bestPose = null;

        for (var entry : cameras) {
            if (!entry.camera.isConnected()) {
                continue;
            }

            var result = entry.estimator.update(entry.camera);
            if (result.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimate = result.get();
            Pose2d pose = estimate.estimatedPose.toPose2d();
            int tagCount = estimate.targetsUsed.size();

            // Sanity gate: reject out-of-field poses
            if (pose.getX() < 0 || pose.getX() > 16.54
                    || pose.getY() < 0 || pose.getY() > 8.21) {
                continue;
            }

            // Sanity gate: reject huge jumps from current odometry
            if (hasSeededPose) {
                double jump = drivetrain.getState().Pose.getTranslation()
                        .getDistance(pose.getTranslation());
                if (jump > 5.0) {
                    continue;
                }
            }

            // Compute ambiguity (use 0 for multi-tag, single-tag uses best target ambiguity)
            double ambiguity = 0.0;
            if (tagCount == 1) {
                ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity();
                if (ambiguity > 0.25) {
                    continue;
                }
            }

            // Track best pose for logging
            if (ambiguity < bestAmbiguity) {
                bestAmbiguity = ambiguity;
                bestPose = pose;
            }

            if (!hasSeededPose) {
                goodReadingCount++;
                drivetrain.resetPose(pose);
                if (goodReadingCount >= SEED_READING_COUNT) {
                    hasSeededPose = true;
                }
                continue;
            }

            // Scale std devs by tag count and ambiguity
            double xyStdDev;
            double thetaStdDev;
            if (tagCount >= 2) {
                // Multi-tag: very trusted
                xyStdDev = 0.01;
                thetaStdDev = Units.degreesToRadians(2);
            } else {
                // Single-tag: scale by ambiguity
                xyStdDev = 0.02 + (ambiguity * 2.0);
                thetaStdDev = Units.degreesToRadians(5 + ambiguity * 50);
            }

            drivetrain.addVisionMeasurement(
                pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
            measurementCount++;
        }

        Logger.recordOutput("Vision/HasTarget", measurementCount > 0 || !hasSeededPose && goodReadingCount > 0);
        Logger.recordOutput("Vision/MeasurementCount", measurementCount);
        Logger.recordOutput("Vision/BestAmbiguity", bestAmbiguity);
        Logger.recordOutput("Vision/PoseSeeded", hasSeededPose);
        if (bestPose != null) {
            Logger.recordOutput("Vision/EstimatedPose", bestPose);
        }
    }

    @Override
    public void periodic() {
        processCamera();
    }
}