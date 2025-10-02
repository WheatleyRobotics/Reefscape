package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages dynamic autonomous path selection and execution based on dashboard settings. Allows for
 * multiple path sections with coral scoring positions.
 */
public class DynamicAuto {
  private static final int MAX_CORAL_SECTIONS = 4;
  private static final int MAX_CORAL_ZONES = 12;
  private static final double raiseTimeBuffer = 0.8;

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
      int j;
      for (j = 0; j < MAX_CORAL_ZONES; j++) {
        sectionChooser.addOption("Branch " + j, j);
      }
      int branch = j + 7;
      sectionChooser.setDefaultOption("Branch " + branch, branch);
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
      Integer branchID = coralSectionChoosers.get(0).getSelected();
      if (branchID == null || branchID == -1) {
        return Commands.none();
      }

      SourcePosition sourcePosition = sourcePositionChooser.getSelected();
      if (sourcePosition == null) {
        errorAlert.set(true);
        return Commands.none();
      }

      StartingPosition startPosition = startingPositionChooser.getSelected();
      if (startPosition == null) {
        errorAlert.set(true);
        return Commands.none();
      }

      String startPathName = startPosition.getPathName() + "-" + branchID;
      PathPlannerPath startToReefPath = loadPath(startPathName);

      if (startToReefPath == null) {
        return Commands.none();
      }

      String reefToSource = branchID + "-" + sourcePosition.getPathName();
      PathPlannerPath reefToSourcePath = loadPath(reefToSource);

      if (reefToSourcePath == null) {
        System.out.println("Second path not found");
        return Commands.none();
      }

      double startToReefPathRaiseTime =
          startToReefPath.getIdealTrajectory(DriveConstants.ppConfig).get().getTotalTimeSeconds()
              - TrapezoidalUtil.calculateTimeToSetpoint(
                  SuperstructureState.L4_CORAL.getValue().getPose().elevatorHeight().getAsDouble(),
                  0,
                  0,
                  Elevator.maxVelocityMetersPerSec.getAsDouble(),
                  Elevator.maxAccelerationMetersPerSec2.getAsDouble());

      startToReefPathRaiseTime = startToReefPathRaiseTime > 0 ? startToReefPathRaiseTime : 0;
      System.out.println(startToReefPathRaiseTime);
      boolean isRightSide = !(branchID % 2 == 0);
      section1 =
          Commands.sequence(
              AutoBuilder.resetOdom(startToReefPath.getStartingHolonomicPose().get()),
              Commands.parallel(
                  Commands.waitSeconds(startToReefPathRaiseTime - raiseTimeBuffer)
                      .andThen(
                          superstructure
                              .runGoal(SuperstructureState.L4_CORAL)
                              .until(superstructure::atGoal)),
                  AutoBuilder.followPath(startToReefPath)),
              AutoScore.getAutoScoreCommand(
                  () -> SuperstructureState.L4_CORAL, isRightSide, drive, superstructure),
              AutoBuilder.followPath(reefToSourcePath)
                  .deadlineFor(superstructure.runGoal(SuperstructureState.INTAKE)),
              superstructure
                  .runGoal(SuperstructureState.INTAKE)
                  .until(superstructure::isHasCoral)
                  .withTimeout(3)
                  .deadlineFor(
                      Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.15, 0, 0)))));
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
    Integer branchID = chooser.getSelected();
    SourcePosition sourcePosition = sourcePositionChooser.getSelected();

    if (branchID == null
        || branchID == -1
        || sourcePosition == null
        || sourcePosition == SourcePosition.NONE) {
      return Commands.none();
    }

    try {
      boolean isRightSide = !(branchID % 2 == 0);

      String sourceToReefName = sourcePosition.getPathName() + "-" + branchID;
      PathPlannerPath sourceToReefPath = loadPath(sourceToReefName);

      if (sourceToReefPath == null) {
        return Commands.none();
      }

      double sourceToReefPathRaiseTime =
          sourceToReefPath.getIdealTrajectory(DriveConstants.ppConfig).get().getTotalTimeSeconds()
              - TrapezoidalUtil.calculateTimeToSetpoint(
                  SuperstructureState.L4_CORAL.getValue().getPose().elevatorHeight().getAsDouble(),
                  0,
                  0,
                  Elevator.maxVelocityMetersPerSec.getAsDouble(),
                  Elevator.maxAccelerationMetersPerSec2.getAsDouble());

      Command sectionCommand =
          Commands.sequence(
              Commands.parallel(
                  AutoBuilder.followPath(sourceToReefPath),
                  Commands.waitSeconds(sourceToReefPathRaiseTime - raiseTimeBuffer)
                      .andThen(
                          superstructure
                              .runGoal(SuperstructureState.L4_CORAL)
                              .until(superstructure::atGoal))),
              AutoScore.getAutoScoreCommand(
                  () -> SuperstructureState.L4_CORAL, isRightSide, drive, superstructure));

      if (!isLast) {
        String pathToSourceName = branchID + "-" + sourcePosition.getPathName();
        PathPlannerPath pathToSource = loadPath(pathToSourceName);

        if (pathToSource != null) {
          Command pathBackToSource =
              AutoBuilder.followPath(pathToSource)
                  .deadlineFor(
                      superstructure
                          .runGoal(SuperstructureState.INTAKE)
                          .onlyIf(() -> !superstructure.isHasCoral()));

          Command waitAtSource =
              superstructure
                  .runGoal(SuperstructureState.INTAKE)
                  .until(superstructure::isHasCoral)
                  .withTimeout(2)
                  .deadlineFor(
                      Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.15, 0, 0))));

          sectionCommand = Commands.sequence(sectionCommand, pathBackToSource, waitAtSource);
        }
      }

      return sectionCommand.alongWith(new PrintCommand("Section " + branchID));
    } catch (Exception e) {
      logError("Error building section for target " + branchID, e);
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
      return PathPlannerPath.fromChoreoTrajectory(pathName);
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
}
