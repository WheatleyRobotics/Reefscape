package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoScore {

  public static final LoggedTunableNumber minClearReefDistance =
      new LoggedTunableNumber("AutoScore/MinClearReefDistance", 0.7);
  public static final LoggedTunableNumber l4Offset =
      new LoggedTunableNumber("AutoScore/L4Offset", 0.6);
  public static final LoggedTunableNumber coralOffset =
      new LoggedTunableNumber("AutoScore/coralOffset", 0.57);

  @AutoLogOutput(key = "AutoScore/StartPose")
  public static Pose2d startPose = new Pose2d();

  public static Command getAutoScoreCommand(
      Supplier<SuperstructureState> state,
      boolean right,
      Drive drive,
      Superstructure superstructure) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotState.getInstance().setShouldTrigSolve(true)),
        /*
        superstructure
            .runGoal(SuperstructureState.INTAKE)
            .onlyIf(() -> !superstructure.isHasCoral()),

         */
        /*
        superstructure
            .runGoal(
                () -> {
                  RobotState.getInstance().setHasDesiredState(false);
                  if (state.get().equals(SuperstructureState.STOW))
                    return SuperstructureState.L1_CORAL;
                  else return state.get();
                })
            .deadlineFor(
                new DriveToPose(
                    drive,
                    () ->
                        FieldConstants.addOffset(
                            FieldConstants.getBranch(
                                RobotState.getInstance().getCurrentZone(), right),
                            -minClearReefDistance.get()))),

             */
        Commands.runOnce(
            () -> {
              startPose = drive.getPose();
              System.out.println(startPose);
            }),
        new DriveToPose(
                drive,
                () ->
                    FieldConstants.addOffset(
                        FieldConstants.getBranch(RobotState.getInstance().getCurrentZone(), right),
                        state.get().equals(SuperstructureState.L4_CORAL)
                            ? -l4Offset.get()
                            : -coralOffset.get()))
            .withTimeout(1),
        // superstructure.runGoal(() -> state.get().getEject()).withTimeout(0.5),
        new WaitCommand(1),
        // RobotState.getInstance().isAuto() ? Commands.none() : getClearReefCommand(drive),
        /*
        new DriveToPose(
            drive,
            () ->
                FieldConstants.addOffset(
                    new Pose2d(
                        AllianceFlipUtil.apply(
                            FieldConstants.Reef.centerFaces[
                                RobotState.getInstance().getCurrentZone().getFace()]
                                .getTranslation()),
                        RobotState.getInstance().getPose().getRotation()),
                    -0.75)),

         */
        // superstructure.runGoal(() -> SuperstructureState.STOW).until(superstructure::atGoal),
        Commands.runOnce(() -> RobotState.getInstance().setShouldTrigSolve(false)));
  }

  public static Command getClearReefCommand(Drive drive) {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-2.5, 0, 0)))
        .withTimeout(0.5)
        .andThen(Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
  }
}
