package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionIOPhoton implements VisionIO {
  PhotonCamera leftCamera = new PhotonCamera("leftCamera");
  Transform3d robotToLeftCamera =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5),
          new Rotation3d(
              0, 0,
              0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.

  PhotonCamera rightCamera = new PhotonCamera("rightCamera");
  Transform3d robotToRightCamera =
      new Transform3d(
          new Translation3d(0.5, 0.0, 0.5),
          new Rotation3d(
              0, 0,
              0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  PhotonPoseEstimator leftPhotonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          leftCamera,
          robotToLeftCamera);
  PhotonPoseEstimator rightPhotonPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          rightCamera,
          robotToRightCamera);

  public Pose3d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    leftPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    rightPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return combineEstimatedPoses(
        leftPhotonPoseEstimator.update(), rightPhotonPoseEstimator.update());
  }

  private Pose3d combineEstimatedPoses(
      Optional<EstimatedRobotPose> pose1, Optional<EstimatedRobotPose> pose2) {
    if (pose1.isEmpty() && pose2.isEmpty()) {
      return new Pose3d();
    } else if (pose1.isEmpty()) {
      return pose2.get().estimatedPose;
    } else if (pose2.isEmpty()) {
      return pose1.get().estimatedPose;
    }
    Pose3d p1 = pose1.get().estimatedPose;
    Pose3d p2 = pose2.get().estimatedPose;
    double x = (p1.getTranslation().getX() + p2.getTranslation().getX()) / 2;
    double y = (p1.getTranslation().getY() + p2.getTranslation().getY()) / 2;
    double z = (p1.getTranslation().getZ() + p2.getTranslation().getZ()) / 2;
    Rotation3d rotation = p1.getRotation().interpolate(p2.getRotation(), 0.5);
    return new Pose3d(x, y, z, rotation);
  }
}
