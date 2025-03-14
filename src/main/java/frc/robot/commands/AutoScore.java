package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class AutoScore {

  public static final LoggedTunableNumber minClearReefDistance =
      new LoggedTunableNumber("AutoScore/MinClearReefDistance", 0.65);
  public static final LoggedTunableNumber l4Offset =
      new LoggedTunableNumber("AutoScore/L4Offset", 0.515);
  public static final LoggedTunableNumber coralOffset =
      new LoggedTunableNumber("AutoScore/coralOffset", 0.51);

  public static Command getAutoScoreCommand(
      Supplier<SuperstructureState> state,
      boolean right,
      Drive drive,
      Superstructure superstructure) {
    return Commands.sequence(
        superstructure
            .runGoal(SuperstructureState.INTAKE)
            .onlyIf(() -> !superstructure.isHasCoral()),
        superstructure
            .runGoal(
                () -> {
                  if (state.get().equals(SuperstructureState.STOW))
                    return SuperstructureState.L1_CORAL;
                  else return state.get();
                })
            .withTimeout(0.6)
            // .until(superstructure::atGoal)
            .deadlineFor(
                new DriveToPose(
                    drive,
                    () ->
                        FieldConstants.addOffset(
                            FieldConstants.getBranch(
                                RobotState.getInstance().getCurrentZone(), right),
                            -minClearReefDistance.get()))),
        Commands.parallel(
            new DriveToPose(
                    drive,
                    () ->
                        FieldConstants.addOffset(
                            FieldConstants.getBranch(
                                RobotState.getInstance().getCurrentZone(), right),
                            state.get().equals(SuperstructureState.L4_CORAL)
                                ? -l4Offset.get()
                                : -coralOffset.get()))
                .withTimeout(2),
            superstructure
                .runGoal(
                    () -> {
                      if (state.get().equals(SuperstructureState.STOW))
                        return SuperstructureState.L1_CORAL;
                      else return state.get();
                    })
                .until(superstructure::atGoal)),
        new WaitCommand(0.1),
        superstructure
            .runGoal(() -> state.get().getEject())
            .until(() -> !superstructure.isHasCoral()),
        Commands.parallel(
            getClearReefCommand(drive),
            new WaitCommand(0.2)
                .andThen(superstructure.runGoal(() -> SuperstructureState.STOW).withTimeout(0.2))));
  }

  public static Command getClearReefCommand(Drive drive) {
    return Commands.sequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1.75, 0, 0)))
            .until(() -> RobotState.getInstance().isClearedReef()),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
  }
}
