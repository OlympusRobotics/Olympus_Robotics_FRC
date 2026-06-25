//this was the file for initializing the cameras running photonvision and has since been depricated after switching to limelight. We never properly callibrated our cameras but the code likely could have worked if one single line was fixed which I will mark out.

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
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark); //Adjusts the cameras coordinate systems and april tag positions to the current seasons

    private final PhotonPoseEstimator poseEstimator;

    /** @deprecated
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
                robotToCamera); //estimates a field position in 3 dimensions for where the robot is based on every different apriltag that the individual camera can see, this is a depricated function
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); //if it can't properly pinpoint a pose using every tag it will use the tag in which it can see the best, depricated function
    }

    /**
     * Update the pose estimator from the latest camera frame.
     *
     * @param camera The PhotonCamera to read from.
     * @return An estimated robot pose if a valid detection exists, otherwise empty.
     */
    public Optional<EstimatedRobotPose> update(PhotonCamera camera) {
        return poseEstimator.update(camera.getLatestResult()); //updates with the most recently collected pose
    }
}