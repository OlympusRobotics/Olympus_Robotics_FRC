package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Camera {
    private PhotonPipelineResult result;
    private PhotonTrackedTarget rightmostTarget;
    private double targetYaw;
    private double targetPitch;
    private double targetArea;
    private Transform3d alternatePose;
    private Transform3d rightCameraToTarget;
    private Pose3d robotpose;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private Transform3d cameraToRobot;
    private Translation2d translation2d;
    private Rotation2d rotation2d;
    private double dumbdistance;

    public record datatatatata(double targetYaw, double targetPitch, double targetArea,
    Transform3d alternatePose, double dumbdistance, Pose3d robotpose, Translation2d translation2d, PhotonPipelineResult result) {}
    
    public datatatatata datata(double dumbdistance2) {
            return new datatatatata(dumbdistance2, dumbdistance2, dumbdistance2, alternatePose, dumbdistance2, robotpose, translation2d, result);
        }

    public datatatatata cameraProcessing(PhotonCamera camera) {
        robotpose = null;
        result = camera.getLatestResult();
        //get information about currently detected targets

        //returns data if there are targets
        if (result.hasTargets()){
            //obtains a list of tracked targets from the result
            rightmostTarget = result.getBestTarget();
            //information from the desired target
        targetYaw = rightmostTarget.getYaw();
        targetPitch = rightmostTarget.getPitch();
        targetArea = rightmostTarget.getArea();
        rotation2d = Rotation2d.fromDegrees(targetYaw);
        //LeBron found out that reprojection error means the distance between a target and the best location for the target to be (the rightmost side)
        alternatePose = rightmostTarget.getAlternateCameraToTarget();
        rightCameraToTarget = rightmostTarget.getBestCameraToTarget();
        
        //This is super long
        if (aprilTagFieldLayout.getTagPose(rightmostTarget.getFiducialId()).isPresent() && cameraToRobot != null) {
            robotpose = PhotonUtils.estimateFieldToRobotAprilTag(rightmostTarget.getBestCameraToTarget(), 
            aprilTagFieldLayout.getTagPose(rightmostTarget.getFiducialId()).get(), cameraToRobot);
        }

        //pythagorean formula for translation2d
        dumbdistance = (Math.hypot(rightCameraToTarget.getX(), rightCameraToTarget.getY()));

        //This gets the distance and rotation from the target
        translation2d = PhotonUtils.estimateCameraToTargetTranslation(dumbdistance, rotation2d);
        return new datatatatata(targetYaw, targetPitch, targetArea, alternatePose, dumbdistance, robotpose, translation2d, result);
        }
    

    //supposedly returns null maybe probably
    return new datatatatata(targetYaw, targetPitch, targetArea, alternatePose, dumbdistance, robotpose, translation2d, result);      
}
public void setCameraToRobot(Transform3d transform) {
    this.cameraToRobot = transform;
}

}