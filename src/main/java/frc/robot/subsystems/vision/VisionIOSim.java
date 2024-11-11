package frc.robot.subsystems.vision;

import frc.lib.structs.VisionPose;
import org.photonvision.PhotonCamera;

public class VisionIOSim implements VisionIO {

  private PhotonCamera leftCam = new PhotonCamera("leftCamera");
  private PhotonCamera rightCam = new PhotonCamera("rightCamera");

  @Override
  public void updateIOInputs(VisionIOInputs inputs) {}

  @Override
  public VisionPose getVisionPose() {
    return new VisionPose();
  }}
