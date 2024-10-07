// CycleCommand.java
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FSDCommand extends Command {
  private final Drive drive;
  private final Pose2d locationA;
  private final Pose2d locationB;
  private final Command commandAtA;
  private final Command commandAtB;

  private boolean goingToA;
  private Command currentCommand;

  public FSDCommand(
      Drive drive, Pose2d locationA, Pose2d locationB, Command commandAtA, Command commandAtB) {
    this.drive = drive;
    this.locationA = locationA;
    this.locationB = locationB;
    this.commandAtA = commandAtA;
    this.commandAtB = commandAtB;

    goingToA = true; // Start by going to locationA

    System.out.println("FSDCommand constructed");

    addRequirements(drive); // Use the drive subsystem
  }

  @Override
  public void initialize() {
    System.out.println("FSDCommand initialized");
    generateAndFollowPathToTarget(goingToA ? locationA : locationB);
  }

  @Override
  public void execute() {
    System.out.println("FSDCommand executing");
    // Monitor the robot's pose to check if it has reached the target
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = goingToA ? locationA : locationB;

    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    if (distanceToTarget < 0.5) {
      // Reached the target
      // Execute the command associated with the target
      if (goingToA) {
        if (commandAtA != null) {
          commandAtA.schedule();
        }
      } else {
        if (commandAtB != null) {
          commandAtB.schedule();
        }
      }
      // Switch target
      goingToA = !goingToA;
      // Generate and follow path to the new target
      generateAndFollowPathToTarget(goingToA ? locationA : locationB);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    // This command should keep running until canceled
    return false;
  }

  private void generateAndFollowPathToTarget(Pose2d targetPose) {
    // Generate path from current position to targetPose
    System.out.println("Generating path to " + targetPose);
    currentCommand = drive.generatePath(targetPose);
    currentCommand.schedule();
  }
}
