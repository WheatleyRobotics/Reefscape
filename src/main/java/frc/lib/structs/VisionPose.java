package frc.lib.structs;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Data;

@Data
public class VisionPose {
  private Pose2d pose2D;
  private double timestamp;

  public VisionPose() {
    this.pose2D = new Pose2d();
    this.timestamp = 0;
  }

  public VisionPose(Pose2d pose, double timestamp) {
    this.pose2D = pose;
    this.timestamp = timestamp;
  }
}
