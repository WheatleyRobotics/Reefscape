package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class AutoCycle {

  private static LoggedTunableNumber offset = new LoggedTunableNumber("AutoCycle", -0.6);
  public static PathConstraints pathConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  public static Queue<Pair<Destinations, SuperstructureState>> cycles = new LinkedList<>();
  private static boolean[] branches = {
    false, false, false, false, false, false, false, false, false, false, false, false
  }; // branch 0 - 11

  private Drive drive;
  private Superstructure superstructure;

  public AutoCycle(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
  }

  public static void addCycle(Destinations destinations, SuperstructureState superstructureState) {
    cycles.add(Pair.of(destinations, superstructureState));
  }

  public void setUpWidget() {
    for (int i = 0; i < branches.length; i++) {
      SmartDashboard.putBoolean("Branches " + i, branches[i]);
    }
    SmartDashboard.putNumber("Level", 4);
    SmartDashboard.putString("AutoCycle", "");
  }

  public static void updateTodo() {
    String todo = cycles.toString();
    SmartDashboard.putString("AutoCycle", todo);
  }

  /**
   * @param right whether to go to the right source
   */
  public Command nextCycle(boolean right) {
    return Commands.defer(
        () -> {
          Pair<Destinations, SuperstructureState> cycle = cycles.poll();

          if (cycle == null) {
            return Commands.none();
          }

          return Commands.sequence(
              superstructure.isHasCoral()
                  ? AutoIntake.getAutoIntakeCommand(drive, superstructure, right)
                  : Commands.none(),
              AutoBuilder.pathfindToPose(cycle.getFirst().getPose(), pathConstraints),
              AutoScore.getAutoScoreCommand(cycle::getSecond, right, drive, superstructure));
        },
        Set.of(drive, superstructure));
  }

  @Getter
  @RequiredArgsConstructor
  public enum Destinations {
    B0(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z0, true), offset.getAsDouble(), 0)),
    B1(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z1, false), offset.getAsDouble(), 0)),
    B2(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z1, true), offset.getAsDouble(), 0)),
    B3(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z2, false), offset.getAsDouble(), 0)),
    B4(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z2, true), offset.getAsDouble(), 0)),
    B5(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z3, false), offset.getAsDouble(), 0)),
    B6(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z3, true), offset.getAsDouble(), 0)),
    B7(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z4, false), offset.getAsDouble(), 0)),
    B8(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z4, true), offset.getAsDouble(), 0)),
    B9(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z5, false), offset.getAsDouble(), 0)),
    B10(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z5, true), offset.getAsDouble(), 0)),
    B11(
        FieldConstants.addOffset(
            FieldConstants.getBranch(RobotState.Zones.Z0, false), offset.getAsDouble(), 0));
    public final Pose2d pose;
  }

  public static void periodic() {
    updateTodo();
    for (int i = 0; i < branches.length; i++) {
      boolean sdValue = SmartDashboard.getBoolean("Branches " + i, branches[i]);
      if (branches[i] != sdValue) {
        SmartDashboard.putBoolean("Branches " + i, false);
        String branchEnum = "B" + i;
        int level = (int) SmartDashboard.getNumber("Level", -1);
        SuperstructureState state = SuperstructureState.valueOf("L" + level + "_CORAL");
        addCycle(Destinations.valueOf(branchEnum), state);
      }
    }
  }
}
