package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.ArrayList;

public class DynamicAuto {
  private final Alert errorAlert = new Alert("Invalid dynamic auto", Alert.AlertType.kError);

  private final SendableChooser<String> startingChooser = new SendableChooser<>();
  private final SendableChooser<String> sourceChooser = new SendableChooser<>();
  private final ArrayList<SendableChooser<Integer>> coralChooser = new ArrayList<>();
  private Superstructure superstructure;
  private Drive drive;

  public DynamicAuto(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;
    startingChooser.addOption("Left", "LEFT");
    startingChooser.addOption("Center", "MIDDLE");
    startingChooser.addOption("Right", "RIGHT");
    sourceChooser.addOption("Left", "LSOURCE");
    sourceChooser.addOption("Right", "RSOURCE");
    for (int i = 0; i < 4; i++) {
      SendableChooser<Integer> chooser = new SendableChooser<>();
      for (int j = 0; j < 12; j++) {
        chooser.addOption("None", -1);
        chooser.addOption("Branch " + j, j);
      }
      coralChooser.add(chooser);
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
      boolean right = !(coralChooser.get(0).getSelected() % 2 == 0);
      s1 =
          Commands.sequence(
              AutoBuilder.resetOdom(
                  PathPlannerPath.fromChoreoTrajectory(
                          startingChooser.getSelected() + "-" + getCoralZone(target))
                      .getStartingHolonomicPose()
                      .get()),
              AutoBuilder.followPath(
                  PathPlannerPath.fromChoreoTrajectory(
                      startingChooser.getSelected() + "-" + getCoralZone(target))),
              AutoScore.getAutoScore(
                  () -> SuperstructureState.L4_CORAL, right, drive, superstructure),
              AutoBuilder.followPath(
                  PathPlannerPath.fromChoreoTrajectory(
                      getCoralZone(target) + "-" + sourceChooser.getSelected())));
    } catch (Exception e) {
      System.out.println(e.toString());
      System.out.println(
          "Error in " + startingChooser.getSelected() + " to " + coralChooser.get(0).getSelected());
      errorAlert.set(true);
      return Commands.none();
    }

    Command s2 = buildSection(coralChooser.get(1).getSelected());
    Command s3 = buildSection(coralChooser.get(2).getSelected());
    Command s4 = buildSection(coralChooser.get(3).getSelected());

    return Commands.sequence(s1, s2, s3, s4);
  }

  private Command buildSection(int target) {
    try {
      String targetString = getCoralZone(target);
      if (targetString.equals("NONE")) {
        return Commands.none();
      }
      boolean right = !(target % 2 == 0);
      return Commands.sequence(
          AutoBuilder.followPath(
              PathPlannerPath.fromChoreoTrajectory(
                  sourceChooser.getSelected() + "-" + targetString)),
          AutoScore.getAutoScore(() -> SuperstructureState.L4_CORAL, right, drive, superstructure),
          AutoBuilder.followPath(
              PathPlannerPath.fromChoreoTrajectory(
                  targetString + "-" + sourceChooser.getSelected())));
    } catch (Exception e) {
      System.out.println("Error in section " + target);
      errorAlert.set(true);
      return Commands.none();
    }
  }

  private String getCoralZone(int id) {
    return switch (id) {
      case 0, 11 -> "Z1";
      case 1, 2 -> "Z2";
      case 3, 4 -> "Z3";
      case 5, 6 -> "Z4";
      case 7, 8 -> "Z5";
      case 9, 10 -> "Z6";
      default -> "NONE";
    };
  }
}
