package frc.robot.subsystems;
// NOTE: Changes to camera names, positions, or vision config must be reflected in Theseus/README.md (Vision section).

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

public class CameraUsing extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean hasSeededPose = false;
    private static final int SEED_READING_COUNT = 5;
    private static final double SEED_TIMEOUT_SECONDS = 5.0;
    private int goodReadingCount = 0;
    private double firstEnableTimestamp = -1;
    private static final double CAM_YAW_OFFSET = 0.256;

    private record CameraEntry(PhotonCamera camera, Camera estimator) {}
    private final List<CameraEntry> cameras;

    private static final String USE_APRIL_ROTATION_KEY = "Use April Rotation";

    // Starting poses per alliance + station (field coordinates, WPILib convention).
    // Blue faces 0° (toward red wall), Red faces 180° (toward blue wall).
    // Stations 1-3 are numbered right-to-left from each alliance's perspective.
    private static final double FIELD_LENGTH = 16.54;
    private static final double START_INSET = 0.75; // distance from wall
    private static final Map<String, Pose2d> STARTING_POSES = Map.of(
        "Blue1", new Pose2d(START_INSET, 1.0, Rotation2d.kZero),
        "Blue2", new Pose2d(START_INSET, 4.1, Rotation2d.kZero),
        "Blue3", new Pose2d(START_INSET, 7.2, Rotation2d.kZero),
        "Red1",  new Pose2d(FIELD_LENGTH - START_INSET, 7.2, Rotation2d.k180deg),
        "Red2",  new Pose2d(FIELD_LENGTH - START_INSET, 4.1, Rotation2d.k180deg),
        "Red3",  new Pose2d(FIELD_LENGTH - START_INSET, 1.0, Rotation2d.k180deg)
    );

    public CameraUsing(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putBoolean(USE_APRIL_ROTATION_KEY, false);

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
        boolean useAprilRotation = SmartDashboard.getBoolean(USE_APRIL_ROTATION_KEY, false);

        // Track first enable time for seed timeout
        if (!hasSeededPose && DriverStation.isEnabled() && firstEnableTimestamp < 0) {
            firstEnableTimestamp = Timer.getFPGATimestamp();
        }

        // If seeding hasn't completed within timeout after first enable, fall back
        // to alliance/station starting pose so the robot doesn't stay at (0,0,0).
        if (!hasSeededPose && firstEnableTimestamp > 0
                && Timer.getFPGATimestamp() - firstEnableTimestamp > SEED_TIMEOUT_SECONDS) {
            Pose2d fallback = getStartingPose();
            drivetrain.resetPose(fallback);
            Logger.recordOutput("Vision/SeedFallback", fallback);
            hasSeededPose = true;
        }

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
                // Multi-tag: very trusted for translation
                xyStdDev = 0.01;
                thetaStdDev = useAprilRotation ? Units.degreesToRadians(2) : Double.MAX_VALUE;

                // Hard-reset if multi-tag shows large translation drift from odometry
                double delta = drivetrain.getState().Pose.getTranslation()
                        .getDistance(pose.getTranslation());
                if (delta > 0.5) {
                    Pose2d resetPose = useAprilRotation
                            ? pose
                            : new Pose2d(pose.getTranslation(), drivetrain.getState().Pose.getRotation());
                    drivetrain.resetPose(resetPose);
                    measurementCount++;
                    continue;
                }
            } else {
                // Single-tag: scale by ambiguity
                xyStdDev = 0.02 + (ambiguity * 2.0);
                thetaStdDev = useAprilRotation
                        ? Units.degreesToRadians(5 + ambiguity * 50)
                        : Double.MAX_VALUE;
            }

            // Replace vision rotation with gyro rotation when not trusting AprilTag heading
            Pose2d visionPose = useAprilRotation
                    ? pose
                    : new Pose2d(pose.getTranslation(), drivetrain.getState().Pose.getRotation());

            drivetrain.addVisionMeasurement(
                visionPose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
            measurementCount++;
        }

        Logger.recordOutput("Vision/HasTarget", measurementCount > 0 || !hasSeededPose && goodReadingCount > 0);
        Logger.recordOutput("Vision/MeasurementCount", measurementCount);
        Logger.recordOutput("Vision/BestAmbiguity", bestAmbiguity);
        Logger.recordOutput("Vision/PoseSeeded", hasSeededPose);
        Logger.recordOutput("Vision/UseAprilRotation", useAprilRotation);
        if (bestPose != null) {
            Logger.recordOutput("Vision/EstimatedPose", bestPose);
        }
    }

    /**
     * Returns the starting pose based on alliance color and driver station number.
     * Falls back to field center facing 0° if alliance/station is unknown.
     */
    private Pose2d getStartingPose() {
        String alliance = DriverStation.getAlliance()
                .map(a -> a == DriverStation.Alliance.Red ? "Red" : "Blue")
                .orElse("Blue");
        int station = DriverStation.getLocation().orElse(2);
        String key = alliance + station;
        return STARTING_POSES.getOrDefault(key,
                new Pose2d(FIELD_LENGTH / 2, 4.1, Rotation2d.kZero));
    }

    @Override
    public void periodic() {
        processCamera();
    }
}