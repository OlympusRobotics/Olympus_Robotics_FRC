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

/** @deprecated 
 * Program that made use of TriftyCams and OrangePis to do field localization.
 * No longer used in favor of Limelights
*/
public class CameraUsing extends SubsystemBase {
    private final Drivetrain drivetrain;
    private boolean hasSeededPose = false;
    private static final int SEED_READING_COUNT = 5;
    private static final double SEED_TIMEOUT_SECONDS = 5.0;
    private int goodReadingCount = 0;
    private double firstEnableTimestamp = -1;

    private record CameraEntry(PhotonCamera camera, Camera estimator) {}
    private final List<CameraEntry> cameras;

    private static final String USE_APRIL_ROTATION_KEY = "Use April Rotation";
    private static final String ENABLE_VISION_KEY = "Enable Vision";
    private boolean enableVision = false;

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
    ); //Doug Horner wrote this and this could definitely be another part of why it didn't work

    /** @deprecated */
    public CameraUsing(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putBoolean(ENABLE_VISION_KEY, enableVision);
        SmartDashboard.putBoolean(USE_APRIL_ROTATION_KEY, false);

        // Physical camera positions relative to robot center and ground (robot-to-camera transforms)
        var robotToCamFL = new Transform3d(
                new Translation3d(-.15, .3048, .4826),
                new Rotation3d(0, 0, Math.PI/4));
        var robotToCamFR = new Transform3d(
                new Translation3d(-.15, -.3048, .4826),
                new Rotation3d(0, 0, -Math.PI/4));
        var robotToCamBL = new Transform3d(
                new Translation3d(-.3302, .2286, .152),
                new Rotation3d(0, Math.toRadians(-14), 3*(Math.PI)/4));
        var robotToCamBR = new Transform3d(
                new Translation3d(-.3302, -.2286, .152),
                new Rotation3d(0, Math.toRadians(-14), -3*(Math.PI)/4));

        var camFL = new PhotonCamera("CameraFL"); //Defining each camera as a new object
        var camFR = new PhotonCamera("CameraFR");
        var camBL = new PhotonCamera("CameraBL");
        var camBR = new PhotonCamera("CameraBR");

        cameras = List.of(
            new CameraEntry(camFL, new Camera(camFL, robotToCamFL)),
            new CameraEntry(camFR, new Camera(camFR, robotToCamFR)),
            new CameraEntry(camBL, new Camera(camBL, robotToCamBL)),
            new CameraEntry(camBR, new Camera(camBR, robotToCamBR))
        ); //allows for going through each camera with one list
    }

    private void processCamera() {
        enableVision = SmartDashboard.getBoolean(ENABLE_VISION_KEY, false); //checks if the button has been clicked on Smartdashboard/Elastic

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

        // When vision is disabled, skip all camera processing (seeding + measurements), allows for code to run much faster
        if (!enableVision) {
            Logger.recordOutput("Vision/EnableVision", false);
            SmartDashboard.putBoolean(ENABLE_VISION_KEY, enableVision);
            return;
        }

        int measurementCount = 0;
        double bestAmbiguity = 1.0;
        Pose2d bestPose = null;
        boolean useAprilRotation = SmartDashboard.getBoolean(USE_APRIL_ROTATION_KEY, false); //checks if the button has been clicked on Smartdashboard/Elastic

        for (var entry : cameras) { //Goes through every camera
            boolean connected = entry.camera.isConnected();
            Logger.recordOutput("Vision/" + entry.camera.getName() + "/Connected", connected);
            if (!connected) {
                continue;
            } //moves onto the next camera if the current camera can not be found

            var result = entry.estimator.update(entry.camera);
            if (result.isEmpty()) {
                continue;
            } //moves onto the next camera if the current camera doesn't have an estimated pose

            EstimatedRobotPose estimate = result.get(); //updates the estimated pose to the one just recieved
            Pose2d pose = estimate.estimatedPose.toPose2d(); //since we don't care about the Z axis translate the pose to 2D
            int tagCount = estimate.targetsUsed.size(); //gets how many apriltags the camera used to create the estimated pose

            // Sanity gate: reject out-of-field poses
            if (pose.getX() < 0 || pose.getX() > 16.54
                    || pose.getY() < 0 || pose.getY() > 8.21) {
                continue;
            } //if out of bounds of the arena move onto the next camera and ignore the collected values

            // Sanity gate: reject huge jumps from current odometry
            if (hasSeededPose) {
                double jump = drivetrain.getState().Pose.getTranslation()
                        .getDistance(pose.getTranslation()); //checks the distance from the pose that the drivetrain is recording through kinematics and the pose that the camera is returning
                if (jump > 5.0) {
                    continue; //since the drivetrain is likely more accurate to be within a meter of the actual pose, if the camera is reading 5 meters away from what the drivetrain is reading ignore it and move onto the next camera
                }
            }

            // Compute ambiguity (use 0 for multi-tag, single-tag uses best target ambiguity)
            double ambiguity = 0.0; //resets the ambiguity to 0 so that the previous cameras isn't used
            if (tagCount == 1) {
                ambiguity = estimate.targetsUsed.get(0).getPoseAmbiguity(); //uses the target farthest to the left, photonvision usually calls this the "best target" even though it has no correlation with how good the target is
                if (ambiguity > 0.25) {
                    continue; //if the camera is unsure of how accurate its pose is (looking at an angle or very far away) by at least 25%, ignore the pose
                }
            }

            // Track best pose for logging
            if (ambiguity < bestAmbiguity) {
                bestAmbiguity = ambiguity; //filter out to find the camera that can see the apriltag to the furthest left the best
                bestPose = pose; //use the pose of the best camera
            }

            if (!hasSeededPose) {
                goodReadingCount++;
                drivetrain.resetPose(pose);
                if (goodReadingCount >= SEED_READING_COUNT) {
                    hasSeededPose = true;
                }
                continue;
            }

            // Scale standard deviations (std devs) by tag count and ambiguity
            double xyStdDev;
            double thetaStdDev;
            if (tagCount >= 2) {
                // Multi-tag: very trusted for translation
                xyStdDev = 0.01;
                thetaStdDev = useAprilRotation ? Units.degreesToRadians(2) : Double.MAX_VALUE; //if its true follow the question mark otherwise follow the colon

                // Hard-reset if multi-tag shows large translation drift from odometry
                double delta = drivetrain.getState().Pose.getTranslation()
                        .getDistance(pose.getTranslation());
                if (delta > 0.5) {
                    Pose2d resetPose = useAprilRotation
                            ? pose
                            : new Pose2d(pose.getTranslation(), drivetrain.getState().Pose.getRotation());
                    drivetrain.resetPose(resetPose); //this basically is resetting the position that the wheel kinematics think the robot is to what the camera thinks it is if the camera can be trusted
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
                    : new Pose2d(pose.getTranslation(), drivetrain.getState().Pose.getRotation()); //the gyroscope is known to be extremely accurate in most scenarios while ambiguous apriltags can mess up the rotation a lot so there is an option to stop trusting rotation from apriltags

            drivetrain.addVisionMeasurement(
                visionPose,
                estimate.timestampSeconds, //this line right here is what I think messed up the position estimation, this needs to be time since last update I believe 
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev) //how much do we trust this pose, higher values mean don't update while lower values mean always update
            ); 
            measurementCount++;
        }

        // Coprocessor-level connection: L = FL+BL, R = FR+BR, logging and debugging mumbo jumbo
        boolean coprocessorL = cameras.get(0).camera.isConnected() || cameras.get(2).camera.isConnected();
        boolean coprocessorR = cameras.get(1).camera.isConnected() || cameras.get(3).camera.isConnected();
        Logger.recordOutput("Vision/CoprocessorL_Connected", coprocessorL);
        Logger.recordOutput("Vision/CoprocessorR_Connected", coprocessorR);

        Logger.recordOutput("Vision/HasTarget", measurementCount > 0 || !hasSeededPose && goodReadingCount > 0);
        Logger.recordOutput("Vision/MeasurementCount", measurementCount);
        Logger.recordOutput("Vision/BestAmbiguity", bestAmbiguity);
        Logger.recordOutput("Vision/PoseSeeded", hasSeededPose);
        Logger.recordOutput("Vision/UseAprilRotation", useAprilRotation);
        Logger.recordOutput("Vision/EnableVision", true);
        SmartDashboard.putBoolean(ENABLE_VISION_KEY, enableVision);
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
    public void periodic() { //uncomment processCamera and the robot will do the camera calculations every 20 ms unless something is taking too long but most of this stuff happens in microseconds and should be fine.
        //processCamera();
    }
}