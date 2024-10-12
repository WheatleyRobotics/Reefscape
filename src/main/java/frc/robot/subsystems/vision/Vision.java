package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.structs.VisionPose;
import frc.robot.RobotObserver;
import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
  private final List<VisionIO> io;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  public Vision() {
    this.io = List.of(new VisionIOSim());
  }

  public Vision(VisionIO io) {
    this.io = List.of(io);
  }

  public Vision(List<VisionIO> io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    RobotObserver.visionPose = getCombinedPose();
  }

  public List<VisionPose> getPoses() {
    return io.stream()
        .collect(ArrayList::new, (list, io) -> list.add(io.getVisionPose()), ArrayList::addAll);
  }

  public VisionPose getCombinedPose() {
    List<VisionPose> poses = getPoses();
    if (poses.isEmpty()) {
      return new VisionPose(); // Return an identity pose if no poses are available
    }

    // Sum up all the x, y coordinates and the angles
    double totalX = 0;
    double totalY = 0;
    double sinSum = 0;
    double cosSum = 0;
    double timestampSum = 0;

    for (VisionPose pose : poses) {
      Translation2d translation = pose.getPose2D().getTranslation();
      totalX += translation.getX();
      totalY += translation.getY();

      // For averaging angles, use sin and cos components
      Rotation2d rotation = pose.getPose2D().getRotation();
      sinSum += Math.sin(rotation.getRadians());
      cosSum += Math.cos(rotation.getRadians());
      timestampSum += pose.getTimestamp();
    }

    // Compute the average x and y
    int numPoses = poses.size();
    double avgX = totalX / numPoses;
    double avgY = totalY / numPoses;

    // Compute the average angle using atan2 of the sin and cos sums
    double avgAngle = Math.atan2(sinSum, cosSum);

    // Compute the average timestampSum
    double avgTimestamp = timestampSum / numPoses;

    // Create a new Pose2d with the averaged translation and rotation
    return new VisionPose(
        new Pose2d(new Translation2d(avgX, avgY), new Rotation2d(avgAngle)), avgTimestamp);
  }
}
