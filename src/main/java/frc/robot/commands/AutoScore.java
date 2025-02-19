package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class AutoScore {

  public static Command getAutoScore(
      Supplier<SuperstructureState> state,
      boolean right,
      Drive drive,
      Superstructure superstructure) {
    return Commands.sequence(
        superstructure
            .runGoal(
                () -> {
                  if (state.get().equals(SuperstructureState.STOW))
                    return SuperstructureState.L1_CORAL;
                  else return state.get();
                })
            .until(superstructure::atGoal)
            .deadlineFor(
                new DriveToPose(
                    drive,
                    () ->
                        FieldConstants.addOffset(
                            FieldConstants.getBranch(
                                RobotState.getInstance().getCurrentZone(), right),
                            -0.5))),
        new DriveToPose(
            drive,
            () ->
                FieldConstants.addOffset(
                    FieldConstants.getBranch(RobotState.getInstance().getCurrentZone(), right),
                    -0.4)),
        superstructure.runGoal(() -> state.get().getEject()).withTimeout(0.5),
        getClearReef(drive),
        superstructure.runGoal(SuperstructureState.STOW).until(superstructure::atGoal));
  }

  public static Command getClearReef(Drive drive) {
    return Commands.sequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-2.5, 0, 0)))
            .until(() -> RobotState.getInstance().isClearedReef()),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
  }
}
