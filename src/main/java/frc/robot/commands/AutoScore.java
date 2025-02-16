package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;

public class AutoScore {
  public static final LoggedTunableNumber readySuperstructureOffset =
      new LoggedTunableNumber("AutoScore/ReadySuperstructureOffset", 0.5);
  public static final LoggedTunableNumber ejectOffset =
      new LoggedTunableNumber("AutoScore/EjectOffset", 0.4);

  private AutoScore() {}

  public static Command getAutoScoreCommand(
      FieldConstants.ReefLevel reefLevel, Drive drive, Superstructure superstructure) {

    return Commands.sequence(
        new DriveToPose(
            drive, () -> FieldConstants.getNearestBranch(true, readySuperstructureOffset.get())),
        superstructure.runGoal(levelToState(reefLevel)));
  }

  // Converts scoring level to a Superstructure state
  public static SuperstructureState levelToState(FieldConstants.ReefLevel level) {
    return switch (level) {
      case L1 -> SuperstructureState.L1_CORAL;
      case L2 -> SuperstructureState.L2_CORAL;
      case L3 -> SuperstructureState.L3_CORAL;
      case L4 -> SuperstructureState.L4_CORAL;
    };
  }
}
