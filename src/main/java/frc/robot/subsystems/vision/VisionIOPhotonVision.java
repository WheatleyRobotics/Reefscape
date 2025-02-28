// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Supplier<Rotation2d> rotationSupplier;
  private final PhotonPoseEstimator estimator;
  private final boolean shouldTrigSolve;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name,
      Transform3d robotToCamera,
      Supplier<Rotation2d> rotationSupplier,
      Boolean shouldTrigSolve) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.rotationSupplier = rotationSupplier;
    this.shouldTrigSolve = shouldTrigSolve;
    estimator =
        new PhotonPoseEstimator(
            aprilTagLayout,
            PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }
      estimator.addHeadingData(
          result.getTimestampSeconds(), new Rotation3d(rotationSupplier.get()));
      // Add pose observation
      if (result.hasTargets()) {
        if (shouldTrigSolve) {
          var estimatedRobotPose = estimator.update(result).get();
          Pose3d robotPose = estimatedRobotPose.estimatedPose;
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  0.0, // Ambiguity
                  estimatedRobotPose.targetsUsed.size(), // Tag count
                  totalTagDistance / result.targets.size(), // Average tag distance
                  PoseObservationType.PHOTONVISIONTRIG)); // Observation type
        } else {
          if (result.multitagResult.isPresent()) {
            var multitagResult = result.multitagResult.get();
            // Calculate robot pose
            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            // Calculate average tag distance
            double totalTagDistance = 0.0;
            for (var target : result.targets) {
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            // Add tag IDs
            tagIds.addAll(multitagResult.fiducialIDsUsed);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    multitagResult.estimatedPose.ambiguity, // Ambiguity
                    multitagResult.fiducialIDsUsed.size(), // Tag count
                    totalTagDistance / result.targets.size(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
