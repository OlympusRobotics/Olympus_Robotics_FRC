package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private final PhotonPoseEstimator poseEstimator;

    /**
     * Create a camera processor using PhotonPoseEstimator with multi-tag PNP on the coprocessor,
     * falling back to the lowest-ambiguity single tag when only one tag is visible.
     *
     * @param camera         The PhotonCamera instance.
     * @param robotToCamera  The robot-to-camera transform (physical position on robot).
     */
    public Camera(PhotonCamera camera, Transform3d robotToCamera) {
        poseEstimator = new PhotonPoseEstimator(
                FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Update the pose estimator from the latest camera frame.
     *
     * @param camera The PhotonCamera to read from.
     * @return An estimated robot pose if a valid detection exists, otherwise empty.
     */
    public Optional<EstimatedRobotPose> update(PhotonCamera camera) {
        return poseEstimator.update(camera.getLatestResult());
    }
}