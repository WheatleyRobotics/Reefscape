package frc.robot.commands;

import static frc.robot.Constants.RobotType.COMPBOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotState;

public class DriveController extends Command {
  private static final double linearkP;
  private static final double linearkD;
  private static final double thetakP;
  private static final double thetakD;

  static {
    switch (Constants.getRobotType()) {
      case COMPBOT, DEVBOT -> {
        linearkP = 8.0;
        linearkD = 0.0;
        thetakP = 4.0;
        thetakD = 0.0;
      }
      default -> {
        linearkP = 4.0;
        linearkD = 0.0;
        thetakP = 4.0;
        thetakD = 0.0;
      }
    }
  }

  private final Drive drive;
  private final Pose2d targetPose;
  private final Timer timer = new Timer();

  private final PIDController xController = new PIDController(linearkP, 0.0, linearkD);
  private final PIDController yController = new PIDController(linearkP, 0.0, linearkD);
  private final PIDController thetaController = new PIDController(thetakP, 0.0, thetakD);

  public DriveController(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.targetPose = targetPose;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.restart();

    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d robot = RobotState.getInstance().getEstimatedPose();
    double xFeedback = xController.calculate(robot.getX(), targetPose.getX());
    double yFeedback = yController.calculate(robot.getY(), targetPose.getY());
    double thetaFeedback =
        thetaController.calculate(
            robot.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback,
            yFeedback,
            thetaFeedback,
            Rotation2d.fromDegrees(robot.getRotation().getDegrees())));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return targetPose
            .getTranslation()
            .getDistance(RobotState.getInstance().getEstimatedPose().getTranslation())
        < 0.05;
  }
}
