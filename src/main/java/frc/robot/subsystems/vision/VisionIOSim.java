package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.structs.VisionPose;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {

  private PhotonCamera leftCam = new PhotonCamera("leftCamera");
  private PhotonCamera rightCam = new PhotonCamera("rightCamera");

  private final PhotonPoseEstimator photonEstimatorLeftCam;
  private final PhotonPoseEstimator photonEstimatorRightCam;

  private PhotonCameraSim cameraSimLeftCam;
  private VisionSystemSim visionSimLeftCam;

  private PhotonCameraSim cameraSimRightCam;
  private VisionSystemSim visionSimRightCam;
  private double lastEstTimestamp = 0;
  private AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  TargetModel targetModel1 = new TargetModel(0.5, 0.25);
  TargetModel targetModel2 = new TargetModel(0.5, 0.25);
  TargetModel targetModel3 = new TargetModel(0.5, 0.25);
  TargetModel targetModel4 = new TargetModel(0.5, 0.25);
  TargetModel targetModel5 = new TargetModel(0.5, 0.25);

  Pose3d targetPose1 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d targetPose2 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d targetPose3 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d targetPose4 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  Pose3d targetPose5 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

  VisionTargetSim visionTarget1 = new VisionTargetSim(targetPose1, targetModel1);
  VisionTargetSim visionTarget2 = new VisionTargetSim(targetPose2, targetModel2);
  VisionTargetSim visionTarget3 = new VisionTargetSim(targetPose3, targetModel3);
  VisionTargetSim visionTarget4 = new VisionTargetSim(targetPose4, targetModel4);
  VisionTargetSim visionTarget5 = new VisionTargetSim(targetPose5, targetModel5);

  public VisionIOSim() {
    photonEstimatorLeftCam =
        new PhotonPoseEstimator(
            kTagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            leftCam,
            Constants.robotToLeftCamera);
    photonEstimatorLeftCam.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorRightCam =
        new PhotonPoseEstimator(
            kTagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rightCam,
            Constants.robotToRightCamera);
    photonEstimatorRightCam.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    // Create the vision system simulation which handles cameras and targets on the field.
    visionSimLeftCam = new VisionSystemSim("leftCam");
    visionSimRightCam = new VisionSystemSim("rightCam");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    visionSimLeftCam.addAprilTags(kTagLayout);
    visionSimRightCam.addAprilTags(kTagLayout);
    // Create simulated camera properties. These can be set to mimic your actual camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    cameraSimLeftCam = new PhotonCameraSim(leftCam, cameraProp);
    cameraSimRightCam = new PhotonCameraSim(rightCam, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSimLeftCam.addCamera(cameraSimLeftCam, Constants.robotToLeftCamera);
    visionSimRightCam.addCamera(cameraSimRightCam, Constants.robotToRightCamera);

    cameraSimLeftCam.enableDrawWireframe(true);
    cameraSimRightCam.enableDrawWireframe(true);

    visionSimLeftCam.addVisionTargets(visionTarget1);
    visionSimRightCam.addVisionTargets(visionTarget1);

    visionSimLeftCam.addVisionTargets(visionTarget2);
    visionSimRightCam.addVisionTargets(visionTarget2);

    visionSimLeftCam.addVisionTargets(visionTarget3);
    visionSimRightCam.addVisionTargets(visionTarget3);

    visionSimLeftCam.addVisionTargets(visionTarget4);
    visionSimRightCam.addVisionTargets(visionTarget4);

    visionSimLeftCam.addVisionTargets(visionTarget5);
    visionSimRightCam.addVisionTargets(visionTarget5);

    // Enable the raw and processed streams. These are enabled by default.
    cameraSimLeftCam.enableRawStream(true);
    cameraSimLeftCam.enableProcessedStream(true);

    cameraSimRightCam.enableRawStream(true);
    cameraSimRightCam.enableProcessedStream(true);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSimLeftCam.enableDrawWireframe(true);
    cameraSimRightCam.enableDrawWireframe(true);
  }

  @Override
  public void updateIOInputs(VisionIOInputs inputs) {}

  @Override
  public VisionPose getVisionPose() {
    return new VisionPose();
  }

  public PhotonPipelineResult getLatestResultLeftCam() {
    return leftCam.getLatestResult();
  }

  public PhotonPipelineResult getLatestResultRightCam() {
    return rightCam.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedLeftPose() {
    var visionEst = photonEstimatorLeftCam.update();
    double latestTimestamp = leftCam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugFieldLeftCam()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugFieldLeftCam().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedRightPose() {
    var visionEst = photonEstimatorRightCam.update();
    double latestTimestamp = rightCam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugFieldRightCam()
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugFieldRightCam().getObject("VisionEstimation").setPoses();
          });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  // ----- Simulation
  @Override
  public void visionSimPeriodic(Pose2d robotSimPose) {
    visionSimLeftCam.update(robotSimPose);
    visionSimRightCam.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    visionSimLeftCam.resetRobotPose(pose);
    visionSimRightCam.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugFieldLeftCam() {
    if (!Robot.isSimulation()) return null;
    return visionSimLeftCam.getDebugField();
  }

  public Field2d getSimDebugFieldRightCam() {
    if (!Robot.isSimulation()) return null;
    return visionSimRightCam.getDebugField();
  }
}
