package frc.robot.subsystems.vision;

import frc.lib.structs.VisionPose;

public class VisionIOSim implements VisionIO {
  @Override
  public void updateIOInputs(VisionIOInputs inputs) {}

  @Override
  public VisionPose getVisionPose() {
    return new VisionPose();
  }
}
