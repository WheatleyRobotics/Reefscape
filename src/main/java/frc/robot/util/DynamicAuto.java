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
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Manages dynamic autonomous path selection and execution based on dashboard settings. Allows for
 * multiple path sections with coral scoring positions.
 */
public class DynamicAuto {
  // Constants
  private static final double SOURCE_WAIT_TIME = 0.5;
  private static final int MAX_CORAL_SECTIONS = 4;
  private static final int MAX_CORAL_ZONES = 12;
  private static final boolean IS_CHOREO = true;

  // Dashboard options
  private final SendableChooser<StartingPosition> startingPositionChooser = new SendableChooser<>();
  private final SendableChooser<SourcePosition> sourcePositionChooser = new SendableChooser<>();
  private final List<SendableChooser<Integer>> coralSectionChoosers = new ArrayList<>();

  // Subsystems
  private final Superstructure superstructure;
  private final Drive drive;

  // Diagnostic
  private final Alert errorAlert =
      new Alert("Invalid dynamic auto configuration", Alert.AlertType.kError);
  private double startTime;

  /** Starting position options for autonomous. */
  private enum StartingPosition {
    LEFT("LEFT"),
    MIDDLE("MIDDLE"),
    RIGHT("RIGHT");

    private final String pathName;

    StartingPosition(String pathName) {
      this.pathName = pathName;
    }

    public String getPathName() {
      return pathName;
    }
  }

  /** Source position options for autonomous. */
  private enum SourcePosition {
    LEFT_SOURCE("LSOURCE"),
    RIGHT_SOURCE("RSOURCE"),
    NONE("NONE");

    private final String pathName;

    SourcePosition(String pathName) {
      this.pathName = pathName;
    }

    public String getPathName() {
      return pathName;
    }
  }

  /**
   * Creates a new DynamicAuto command builder.
   *
   * @param drive The drive subsystem
   * @param superstructure The superstructure subsystem
   */
  public DynamicAuto(Drive drive, Superstructure superstructure) {
    this.drive = drive;
    this.superstructure = superstructure;

    // Configure starting position chooser
    startingPositionChooser.addOption("Left", StartingPosition.LEFT);
    startingPositionChooser.addOption("Center", StartingPosition.MIDDLE);
    startingPositionChooser.setDefaultOption("Right", StartingPosition.RIGHT);

    // Configure source position chooser
    sourcePositionChooser.addOption("Left", SourcePosition.LEFT_SOURCE);
    sourcePositionChooser.setDefaultOption("Right", SourcePosition.RIGHT_SOURCE);

    // Configure coral section choosers
    for (int i = 0; i < MAX_CORAL_SECTIONS; i++) {
      SendableChooser<Integer> sectionChooser = new SendableChooser<>();
      sectionChooser.addOption("None", -1);

      for (int j = 0; j < MAX_CORAL_ZONES; j++) {
        sectionChooser.addOption("Branch " + j, j);
      }

      sectionChooser.setDefaultOption("Branch 5", 5);
      coralSectionChoosers.add(sectionChooser);
      SmartDashboard.putData("Auto Coral #" + i, sectionChooser);
    }

    // Add choosers to dashboard
    SmartDashboard.putData("Starting Position", startingPositionChooser);
    SmartDashboard.putData("Source Position", sourcePositionChooser);
  }

  /**
   * Builds and returns the complete autonomous command based on selected options.
   *
   * @return The complete autonomous command sequence
   */
  public Command getAutoCommand() {
    // Get first section (starting position to first coral)
    Command section1;
    try {
      Integer target = coralSectionChoosers.get(0).getSelected();
      if (target == null || target == -1) {
        return Commands.none();
      }

      // Determine if coral is on the right side
      boolean isRightSide = !(target % 2 == 0);
      String coralZone = getCoralZoneName(target);
      StartingPosition startPosition = startingPositionChooser.getSelected();

      if (startPosition == null) {
        errorAlert.set(true);
        return Commands.none();
      }

      String startPathName = startPosition.getPathName() + "-" + coralZone;
      PathPlannerPath startPath = loadPath(startPathName);

      if (startPath == null) {
        return Commands.none();
      }

      SourcePosition sourcePosition = sourcePositionChooser.getSelected();
      if (sourcePosition == null) {
        errorAlert.set(true);
        return Commands.none();
      }

      String secondPathName = coralZone + "-" + sourcePosition.getPathName();
      PathPlannerPath secondPath = loadPath(secondPathName);

      if (secondPath == null) {
        return Commands.none();
      }

      section1 =
          Commands.sequence(
              AutoBuilder.resetOdom(startPath.getStartingHolonomicPose().get()),
              AutoBuilder.followPath(startPath),
              AutoScore.getAutoScoreCommand(
                  () -> SuperstructureState.L4_CORAL, isRightSide, drive, superstructure),
              AutoBuilder.followPath(secondPath)
                  .deadlineFor(superstructure.runGoal(SuperstructureState.INTAKE)),
              superstructure.runGoal(SuperstructureState.INTAKE).withTimeout(SOURCE_WAIT_TIME));
    } catch (Exception e) {
      logError("Error in first section", e);
      return Commands.none();
    }

    // Build remaining sections
    Command section2 = buildSection(coralSectionChoosers.get(1), false);
    Command section3 = buildSection(coralSectionChoosers.get(2), false);
    Command section4 = buildSection(coralSectionChoosers.get(3), true);

    // Combine all sections into a complete autonomous routine
    return Commands.sequence(
        Commands.runOnce(() -> startTime = Timer.getFPGATimestamp()),
        section1,
        section2,
        section3,
        section4,
        Commands.runOnce(
            () ->
                System.out.println(
                    "Auto completed in " + (Timer.getFPGATimestamp() - startTime) + " seconds")));
  }

  /**
   * Builds a section of autonomous commands.
   *
   * @param chooser The chooser for this section
   * @param isLast Whether this is the last section
   * @return The command sequence for this section
   */
  private Command buildSection(SendableChooser<Integer> chooser, boolean isLast) {
    Integer target = chooser.getSelected();
    SourcePosition sourcePosition = sourcePositionChooser.getSelected();

    if (target == null
        || target == -1
        || sourcePosition == null
        || sourcePosition == SourcePosition.NONE) {
      return Commands.none();
    }

    try {
      String coralZone = getCoralZoneName(target);
      boolean isRightSide = !(target % 2 == 0);

      String pathToCoralName = sourcePosition.getPathName() + "-" + coralZone;
      PathPlannerPath pathToCoral = loadPath(pathToCoralName);

      if (pathToCoral == null) {
        return Commands.none();
      }

      Command sectionCommand =
          Commands.sequence(
              AutoBuilder.followPath(pathToCoral)
                  .deadlineFor(
                      superstructure
                          .runGoal(SuperstructureState.INTAKE)
                          .onlyIf(() -> !superstructure.isHasCoral())),

              // Score the coral
              AutoScore.getAutoScoreCommand(
                  () -> SuperstructureState.L4_CORAL, isRightSide, drive, superstructure));

      if (!isLast) {
        String pathToSourceName = coralZone + "-" + sourcePosition.getPathName();
        PathPlannerPath pathToSource = loadPath(pathToSourceName);

        if (pathToSource != null) {
          Command pathBackToSource =
              AutoBuilder.followPath(pathToSource)
                  .deadlineFor(
                      superstructure
                          .runGoal(SuperstructureState.INTAKE)
                          .onlyIf(() -> !superstructure.isHasCoral()));

          Command waitAtSource =
              superstructure.runGoal(SuperstructureState.INTAKE).withTimeout(SOURCE_WAIT_TIME);

          sectionCommand = Commands.sequence(sectionCommand, pathBackToSource, waitAtSource);
        }
      }

      return sectionCommand;
    } catch (Exception e) {
      logError("Error building section for target " + target, e);
      return Commands.none();
    }
  }

  /**
   * Loads a path from PathPlanner or Choreo.
   *
   * @param pathName The name of the path
   * @return The loaded path, or null if loading failed
   */
  private PathPlannerPath loadPath(String pathName) {
    try {
      return IS_CHOREO
          ? PathPlannerPath.fromChoreoTrajectory(pathName)
          : PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      logError("Failed to load path: " + pathName, e);
      return null;
    }
  }

  /**
   * Logs an error with details.
   *
   * @param message The error message
   * @param e The exception
   */
  private void logError(String message, Exception e) {
    System.out.println(message + ": " + e.toString());
    errorAlert.set(true);
  }

  /**
   * Gets the starting pose for the autonomous routine.
   *
   * @return The starting pose
   */
  @AutoLogOutput(key = "DynamicAuto/StartingPose")
  public Pose2d getStartPose() {
    try {
      StartingPosition startPosition = startingPositionChooser.getSelected();
      if (startPosition == null) {
        return new Pose2d();
      }

      // Use Z3 as default for getting start pose
      PathPlannerPath startPath = loadPath(startPosition.getPathName() + "-Z3");
      return startPath != null ? startPath.getStartingHolonomicPose().get() : new Pose2d();
    } catch (Exception e) {
      logError("Error getting start pose", e);
      return new Pose2d();
    }
  }

  /**
   * Converts a coral zone ID to its zone name.
   *
   * @param id The coral zone ID
   * @return The coral zone name
   */
  private String getCoralZoneName(int id) {
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
