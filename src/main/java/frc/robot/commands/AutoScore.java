package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;

public class AutoScore {

  public static Command getAutoScore(
      SuperstructureState state, boolean right, Drive drive, Superstructure superstructure) {
    return Commands.sequence(
        Commands.parallel(
            new DriveToPose(drive, () -> FieldConstants.getNearestBranch(right, -0.6)),
            superstructure.runGoal(state).until(superstructure::atGoal)),
        new DriveToPose(drive, () -> FieldConstants.getNearestBranch(right, -0.4)),
        superstructure.runGoal(SuperstructureState.getEject(state)).withTimeout(0.5),
        new DriveToPose(drive, () -> FieldConstants.getNearestBranch(right, -0.6)),
        superstructure.runGoal(SuperstructureState.STOW).until(superstructure::atGoal));
  }
}
