package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.structs.VisionPose;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d pose = new Pose2d();
  }

  public default void updateIOInputs(VisionIOInputs inputs) {}

  public default VisionPose getVisionPose() {
    return new VisionPose();
  }
}
