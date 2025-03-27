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
      new LoggedTunableNumber("AutoScore/MinClearReefDistance", 0.7);
  public static final LoggedTunableNumber l4Offset =
      new LoggedTunableNumber("AutoScore/L4Offset", 0.5125);
  public static final LoggedTunableNumber coralOffset =
      new LoggedTunableNumber("AutoScore/coralOffset", 0.51);

  public static Command getAutoScoreCommand(
      Supplier<SuperstructureState> state,
      boolean right,
      Drive drive,
      Superstructure superstructure) {
    return Commands.sequence(
        /*
        superstructure
            .runGoal(SuperstructureState.INTAKE)
            .until(superstructure::isHasCoral)
            .onlyIf(() -> !Robot.isSimulation())
            .withTimeout(2),

             */

        Commands.runOnce(() -> RobotState.getInstance().setSide(right ? 1 : 0)),
        superstructure
            .runGoal(
                () -> {
                  if (state.get().equals(SuperstructureState.STOW))
                    return SuperstructureState.L1_CORAL;
                  else return state.get();
                })
            .until(superstructure::atGoal)
            .deadlineFor(
                RobotState.getInstance().isClearedReef()
                    ? new DriveToPose(
                        drive,
                        () ->
                            FieldConstants.addOffset(
                                    FieldConstants.getBranch(
                                        RobotState.getInstance().getCurrentZone(), right),
                                    -minClearReefDistance.get())
                                .interpolate(RobotState.getInstance().getPose(), 0.5))
                    : new DriveToPose(
                        drive,
                        () ->
                            FieldConstants.addOffset(
                                FieldConstants.getBranch(
                                    RobotState.getInstance().getCurrentZone(), right),
                                -minClearReefDistance.get())))
            .onlyIf(() -> !superstructure.getGoal().equals(state.get())),
        Commands.parallel(
                new DriveToPose(
                    drive,
                    () ->
                        FieldConstants.addOffset(
                            FieldConstants.getBranch(
                                RobotState.getInstance().getCurrentZone(), right),
                            state.get().equals(SuperstructureState.L4_CORAL)
                                ? -l4Offset.get()
                                : -coralOffset.get())),
                superstructure
                    .runGoal(
                        () -> {
                          if (state.get().equals(SuperstructureState.STOW))
                            return SuperstructureState.L1_CORAL;
                          else return state.get();
                        })
                    .until(superstructure::atGoal))
            .withTimeout(2),
        new WaitCommand(0.1),
        superstructure
            .runGoal(() -> state.get().getEject())
            .until(() -> !superstructure.isHasCoral())
            .andThen(new WaitCommand(0.1)),
        new WaitCommand(0.2)
            .andThen(Commands.runOnce(() -> superstructure.runGoal(() -> SuperstructureState.STOW)))
            .deadlineFor(
                RobotState.getInstance().isAuto()
                    ? Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0)))
                    : getClearReefCommand(drive)),
        Commands.runOnce(() -> RobotState.getInstance().setSide(-1)));
  }

  public static Command getClearReefCommand(Drive drive) {
    return Commands.sequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.75, 0, 0)))
            .until(() -> RobotState.getInstance().isClearedReef()),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
  }
}
