package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;

public class AutoScore {
  public static Command getAutoScore(boolean right, Drive drive, Superstructure superstructure) {
    return Commands.sequence(
        new DriveToPose(drive, () -> FieldConstants.getNearestBranch(right, -0.4)),
        Commands.runOnce(
            () -> {
              SuperstructureState currentState = superstructure.getState();
              SuperstructureState ejectState = SuperstructureState.getEject(currentState);
              if (!currentState.equals(ejectState)) {
                superstructure.runGoal(SuperstructureState.L4_CORAL_EJECT).schedule();
              }
            },
            superstructure));
  }

  public static Command backOut(Drive drive, Superstructure superstructure) {
    return Commands.sequence(
        Commands.run(
                () -> {
                  drive.runVelocity(new ChassisSpeeds(-1, 0, 0));
                })
            .withTimeout(0.5),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))),
        superstructure.runGoal(SuperstructureState.STOW));
  }
}
