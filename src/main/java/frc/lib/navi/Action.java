package frc.lib.navi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.NonNull;

public record Action(Pose2d targetPose, Command targetCommand) {
  public Action(@NonNull Pose2d targetPose, @NonNull Command targetCommand) {
    this.targetPose = targetPose;
    this.targetCommand = targetCommand;
  }
}
