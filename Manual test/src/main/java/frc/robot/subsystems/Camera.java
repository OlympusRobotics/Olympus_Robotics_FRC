package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Camera {
    private PhotonPipelineResult result;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget rightmostTarget;
    private double targetYaw;
    private double targetPitch;
    private double targetArea;
    private Transform3d pose;
    private Transform3d alternatePose;
    private List<TargetCorner> corners;
    private int targetID;
    private PhotonTrackedTarget target;
    private double poseError;
    private Transform3d rightCameraToTarget;
    private Transform3d alternateCameraToTarget;
    private Pose3d robotpose;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Transform3d cameraToRobot;
    private Translation2d translation2d;
    private Rotation2d rotation2d;
    private Constants robotConstants;
    private double dumbdistance;
    private PhotonPoseEstimator estimatorerorer;

    public record datatatatata(Transform3d pose, AprilTagFieldLayout aprilTagFieldLayout, double targetYaw, double targetPitch, double targetArea,
    Transform3d alternatePose, double dumbdistance, Pose3d robotpose, Translation2d translation2d, PhotonPoseEstimator estimatorerorer, PhotonPipelineResult result) {}
    
    public datatatatata datata(double dumbdistance2) {
            return new datatatatata(pose, aprilTagFieldLayout, dumbdistance2, dumbdistance2, dumbdistance2, alternatePose, dumbdistance2, robotpose, translation2d, estimatorerorer, result);
        }

    public datatatatata cameraProcessing(PhotonCamera camera) {
        robotConstants = new Constants();
        result = camera.getLatestResult();

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        //get information about currently detected targets

        //returns data if there are targets
        if (result.hasTargets()){
            //obtains a list of tracked targets from the result
            targets = result.getTargets();
            rightmostTarget = result.getBestTarget();
            //information from the desired target
        targetYaw = rightmostTarget.getYaw();
        targetPitch = rightmostTarget.getPitch();
        targetArea = rightmostTarget.getArea();
        rotation2d = Rotation2d.fromDegrees(targetYaw);
        //LeBron found out that reprojection error means the distance between a target and the best location for the target to be (the rightmost side)
        pose = rightmostTarget.getBestCameraToTarget();
        alternatePose = rightmostTarget.getAlternateCameraToTarget();
        corners = rightmostTarget.getDetectedCorners();

        targetID = rightmostTarget.getFiducialId();
        poseError = rightmostTarget.getPoseAmbiguity();

        rightCameraToTarget = rightmostTarget.getBestCameraToTarget();
        alternateCameraToTarget = rightmostTarget.getAlternateCameraToTarget();
        
        //Before and after fully processing
        camera.takeInputSnapshot();
        camera.takeOutputSnapshot();
        
        //This is super long
        if (aprilTagFieldLayout.getTagPose(rightmostTarget.getFiducialId()).isPresent()) {
            robotpose = PhotonUtils.estimateFieldToRobotAprilTag(rightmostTarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(rightmostTarget.getFiducialId()).get(), cameraToRobot);
        }

        //pythagorean formula for translation2d
        dumbdistance = (Math.pow(Math.pow(rightCameraToTarget.getX(), 2) + Math.pow(rightCameraToTarget.getY(), 2), .5));

        //No idea if this works
        estimatorerorer = new PhotonPoseEstimator(aprilTagFieldLayout, pose);

        //This gets the distance and rotation from the target
        translation2d = PhotonUtils.estimateCameraToTargetTranslation(dumbdistance, rotation2d);
        return new datatatatata(pose, aprilTagFieldLayout, targetYaw, targetPitch, targetArea, alternatePose, dumbdistance, robotpose, translation2d, estimatorerorer, result);
        }
    

    //supposedly returns null maybe probably
    return new datatatatata(pose, aprilTagFieldLayout, targetYaw, targetPitch, targetArea, alternatePose, dumbdistance, robotpose, translation2d, estimatorerorer, result);      
}


}