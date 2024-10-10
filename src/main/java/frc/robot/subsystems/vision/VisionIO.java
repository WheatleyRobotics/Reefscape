package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d pose = new Pose2d();
  }

  public default void updateIOInputs(VisionIOInputs inputs) {}

  public default Pose2d getPose() {
    return new Pose2d();
  }
}
