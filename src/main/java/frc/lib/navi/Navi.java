package frc.lib.navi;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import lombok.Setter;

public class Navi {
  private final ArrayList<Action> actions = new ArrayList<>();
  private final Drive driveSubsystem;
  private static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
      new PathConstraints(4.0, 4.0, Math.toRadians(360), Math.toRadians(540));
  @Setter private boolean shouldRepeat = false;

  public Navi(Drive driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public void addAction(Action action) {
    actions.add(action);
    System.out.println("Added action to Navi: " + action);
  }

  public Command buildCommandSequence() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    for (Action action : actions) {
      Pose2d targetPose = action.getTargetPose();
      Command navigateCommand = driveSubsystem.generatePath(targetPose, DEFAULT_PATH_CONSTRAINTS);
      Command targetCommand = action.getTargetCommand();
      commandGroup.addCommands(navigateCommand, targetCommand);
    }

    if (shouldRepeat) {
      return commandGroup.repeatedly();
    } else {
      return commandGroup;
    }
  }
}
