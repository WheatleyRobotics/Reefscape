package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimeLight implements VisionIO {
  private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private final double[] table_blue = nt.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

  @Override
  public Pose2d getPose() {
    return new Pose2d(table_blue[0], table_blue[1], new Rotation2d(table_blue[5]));
  }

  @Override
  public void updateIOInputs(VisionIOInputs inputs) {
    inputs.pose = getPose();
  }
}
