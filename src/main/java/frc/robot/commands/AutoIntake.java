package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.LoggedTunableNumber;

public class AutoIntake {

  public static final LoggedTunableNumber stationPathFindOffset =
      new LoggedTunableNumber("AutoIntake/stationPathFindOffset", 0.6);
  public static final LoggedTunableNumber stationDTPOffset =
      new LoggedTunableNumber("AutoIntake/stationDTPOffset", 0.5);
  public static final LoggedTunableNumber stationTranslation =
      new LoggedTunableNumber("AutoIntake/stationTranslation", 0.2);
  public static final LoggedTunableNumber pathFindTolerance =
      new LoggedTunableNumber("AutoIntake/shouldRunPathfind", 0.5);

  public static Command getAutoIntakeCommand(
      Drive drive, Superstructure superstructure, boolean right) {
    return superstructure
        .runGoal(SuperstructureState.INTAKE)
        .until(superstructure::isHasCoral)
        .deadlineFor(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.2, 0, 0))));
    /*
    .deadlineFor(
        Commands.sequence(
            RobotState.getInstance()
                        .getPose()
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.getCorrected(
                                FieldConstants.addOffset(
                                        right
                                            ? FieldConstants.CoralStation.rightCenterFace
                                            : FieldConstants.CoralStation.leftCenterFace,
                                        stationPathFindOffset.getAsDouble(),
                                        stationTranslation.getAsDouble())
                                    .getTranslation()))
                    > pathFindTolerance.getAsDouble()
                ? AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.getCorrected(
                            FieldConstants.addOffset(
                                right
                                    ? FieldConstants.CoralStation.rightCenterFace
                                    : FieldConstants.CoralStation.leftCenterFace,
                                stationPathFindOffset.getAsDouble(),
                                stationTranslation.getAsDouble())),
                        AutoCycle.pathConstraints)
                    .andThen(
                        new DriveToPose(
                            drive,
                            () ->
                                AllianceFlipUtil.getCorrected(
                                    FieldConstants.addOffset(
                                        right
                                            ? FieldConstants.CoralStation.rightCenterFace
                                            : FieldConstants.CoralStation.leftCenterFace,
                                        stationDTPOffset.getAsDouble(),
                                        stationTranslation.getAsDouble()))))
                : Commands.none(),
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.2, 0, 0)))));

         */
  }
}
