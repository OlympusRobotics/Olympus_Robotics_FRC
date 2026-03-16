package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private final AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private Transform3d cameraToRobot;

    /** Result returned from a single camera processing cycle. */
    public record CameraResult(
            Pose3d robotPose,
            double distance,
            double ambiguity,
            PhotonPipelineResult pipelineResult) {
    }

    /**
     * Process a single camera frame and estimate the robot's field pose from the
     * best AprilTag detection.
     *
     * @return CameraResult with pose (null if no valid detection), distance,
     *         ambiguity, and raw pipeline result.
     */
    public CameraResult process(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets() || cameraToRobot == null) {
            return new CameraResult(null, 0, 1.0, result);
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
        double distance = Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
        double ambiguity = bestTarget.getPoseAmbiguity();

        Pose3d robotPose = null;
        var tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
        if (tagPose.isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    cameraToTarget, tagPose.get(), cameraToRobot);
        }

        return new CameraResult(robotPose, distance, ambiguity, result);
    }

    public void setCameraToRobot(Transform3d transform) {
        this.cameraToRobot = transform;
    }
}