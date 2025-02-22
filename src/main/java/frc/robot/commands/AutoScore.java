package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
      boolean clear,
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
        superstructure.runGoal(() -> state.get().getEject()).withTimeout(0.25),
        superstructure
            .runGoal(SuperstructureState.STOW)
            .until(superstructure::atGoal)
            .deadlineFor(getClearReef(drive).onlyIf(() -> clear))); //
  }

  public static Command getClearReef(Drive drive) {
    return Commands.sequence(
        new DriveToPose(
            drive,
            () ->
                FieldConstants.addOffset(
                    FieldConstants.Reef.centerFaces[
                        RobotState.getInstance().getCurrentZone().getFace()]
                        .plus(
                            new Transform2d(new Translation2d(), Rotation2d.fromRadians(Math.PI))),
                    -1)),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
  }
}
