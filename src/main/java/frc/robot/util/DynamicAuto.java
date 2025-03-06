package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;

public class DynamicAuto {
  private static final double intakeTime = .5;
  private final Alert errorAlert = new Alert("Invalid dynamic auto", Alert.AlertType.kError);

  private final SendableChooser<String> startingChooser = new SendableChooser<>();
  private final SendableChooser<String> sourceChooser = new SendableChooser<>();
  private final ArrayList<SendableChooser<Integer>> coralChooser = new ArrayList<>();
  private Superstructure superstructure;
  private Drive drive;
  private final boolean isChoreo = true;
  private double startTime;

  public DynamicAuto(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
    startingChooser.addOption("Left", "LEFT");
    startingChooser.setDefaultOption("Center", "MIDDLE");
    startingChooser.addOption("Right", "RIGHT");
    sourceChooser.addOption("Left", "LSOURCE");
    sourceChooser.setDefaultOption("Right", "RSOURCE");
    for (int i = 0; i < 4; i++) {
      SendableChooser<Integer> chooser = new SendableChooser<>();
      for (int j = 0; j < 12; j++) {
        chooser.addOption("None", -1);
        chooser.addOption("Branch " + j, j);
      }
      coralChooser.add(chooser);
      chooser.setDefaultOption("Branch 5", 5);
    }

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putData("Auto Coral #" + i, coralChooser.get(i));
    }
    SmartDashboard.putData("Starting Position", startingChooser);
    SmartDashboard.putData("Source Position", sourceChooser);
  }

  public Command getAutoCommand() {
    Command s1;
    try {
      int target = coralChooser.get(0).getSelected();
      String source = sourceChooser.getSelected();
      boolean right = !(coralChooser.get(0).getSelected() % 2 == 0);

      PathPlannerPath startPath =
          isChoreo
              ? PathPlannerPath.fromChoreoTrajectory(
                  startingChooser.getSelected() + "-" + getCoralZone(target))
              : PathPlannerPath.fromPathFile(
                  startingChooser.getSelected() + "-" + getCoralZone(target));
      s1 =
          Commands.sequence(
              AutoBuilder.resetOdom(startPath.getStartingHolonomicPose().get()),
              AutoBuilder.followPath(startPath),
              AutoScore.getAutoScoreCommand(
                  () -> SuperstructureState.L4_CORAL, right, drive, superstructure),
              AutoBuilder.followPath(
                  isChoreo
                      ? PathPlannerPath.fromChoreoTrajectory(
                          getCoralZone(target) + "-" + sourceChooser.getSelected())
                      : PathPlannerPath.fromPathFile(
                          getCoralZone(target) + "-" + sourceChooser.getSelected())));
    } catch (Exception e) {
      System.out.println(e.toString());
      System.out.println(
          "Error in " + startingChooser.getSelected() + " to " + coralChooser.get(0).getSelected());
      errorAlert.set(true);
      return Commands.none();
    }

    Command s2 = buildSection(coralChooser.get(1), false);
    Command s3 = buildSection(coralChooser.get(2), false);
    Command s4 = buildSection(coralChooser.get(3), true);

    return Commands.sequence(
        Commands.runOnce(() -> startTime = Timer.getFPGATimestamp()),
        s1,
        s2,
        s3,
        s4,
        Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp() - startTime)));
  }

  private Command buildSection(SendableChooser<Integer> chooser, boolean isLast) {
    if (chooser.getSelected() == null) {
      System.out.println("No target selected");
      return Commands.none();
    }
    if (sourceChooser.getSelected() == null) {
      System.out.println("No source selected");
      return Commands.none();
    }
    int target = chooser.getSelected();
    try {
      String targetString = getCoralZone(target);
      if (targetString.equals("NONE")) {
        return Commands.none();
      }
      if (sourceChooser.getSelected().equals("NONE")) {
        return Commands.none();
      }
      boolean right = !(target % 2 == 0);
      return Commands.sequence(
          new WaitCommand(intakeTime),
          AutoBuilder.followPath(
                  isChoreo
                      ? PathPlannerPath.fromChoreoTrajectory(
                          sourceChooser.getSelected() + "-" + targetString)
                      : PathPlannerPath.fromPathFile(
                          sourceChooser.getSelected() + "-" + targetString))
              .deadlineFor(
                  superstructure
                      .runGoal(SuperstructureState.INTAKE)
                      .until(() -> superstructure.isHasCoral())),
          AutoScore.getAutoScoreCommand(
              () -> SuperstructureState.L4_CORAL, right, drive, superstructure),
          isLast
              ? Commands.none()
              : AutoBuilder.followPath(
                  isChoreo
                      ? PathPlannerPath.fromChoreoTrajectory(
                          targetString + "-" + sourceChooser.getSelected())
                      : PathPlannerPath.fromPathFile(
                          targetString + "-" + sourceChooser.getSelected())));
    } catch (Exception e) {
      System.out.println("Error in section " + target);
      errorAlert.set(true);
      return Commands.none();
    }
  }

  @AutoLogOutput(key = "DynamicAuto/StartingPose")
  public Pose2d getStartPose() {
    try {
      if (startingChooser.getSelected() == null) {
        // System.out.println("No starting position selected");
        return new Pose2d();
      }
      PathPlannerPath startPath =
          isChoreo
              ? PathPlannerPath.fromChoreoTrajectory(startingChooser.getSelected() + "-" + "Z3")
              : PathPlannerPath.fromPathFile(startingChooser.getSelected() + "-" + "Z3");
      return startPath.getStartingHolonomicPose().get();
    } catch (Exception e) {
      errorAlert.set(true);
      return new Pose2d();
    }
  }

  private String getCoralZone(int id) {
    return switch (id) {
      case 0, 11 -> "Z0";
      case 1, 2 -> "Z1";
      case 3, 4 -> "Z2";
      case 5, 6 -> "Z3";
      case 7, 8 -> "Z4";
      case 9, 10 -> "Z5";
      default -> "NONE";
    };
  }
}
