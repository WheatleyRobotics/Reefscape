package frc.robot.commands;

import static frc.robot.Constants.RobotType.COMPBOT;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.Logger;

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
  private Pose2d targetPose;
  private final Timer timer = new Timer();
  private boolean left = false;

  private boolean autoAlign = false;

  private final PIDController xController = new PIDController(linearkP, 0.0, linearkD);
  private final PIDController yController = new PIDController(linearkP, 0.0, linearkD);
  private final PIDController thetaController = new PIDController(thetakP, 0.0, thetakD);

  public DriveController(Pose2d targetPose, Drive drive) {
    this.drive = drive;
    this.targetPose = targetPose;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public DriveController(boolean left, Drive drive) {
    this.drive = drive;
    this.left = left;
    this.autoAlign = true;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {

    if (autoAlign) {
      RobotState.Zones zone = RobotState.getInstance().getCurrentZone();
      double x;
      double y;
      double theta;

      if (AllianceFlipUtil.shouldFlip()) {
        x = AllianceFlipUtil.applyX(FieldConstants.Reef.centerFaces[zone.getFace()].getX());
        y = AllianceFlipUtil.applyY(FieldConstants.Reef.centerFaces[zone.getFace()].getY());
      } else {
        x = FieldConstants.Reef.centerFaces[zone.getFace()].getX();
        y = FieldConstants.Reef.centerFaces[zone.getFace()].getY();
      }

      Translation2d flippedReef;
      if (AllianceFlipUtil.shouldFlip()) {
        flippedReef = AllianceFlipUtil.apply(FieldConstants.Reef.center);
      } else {
        flippedReef = FieldConstants.Reef.center;
      }

      theta =
          Math.atan2(
              RobotState.getInstance().getEstimatedPose().getY() - flippedReef.getY(),
              RobotState.getInstance().getEstimatedPose().getX() - flippedReef.getX());

      theta += Math.PI / 6.0;
      double normalizedAngle = (theta + (2 * Math.PI)) % (2 * Math.PI);
      this.targetPose = new Pose2d(x, y, Rotation2d.fromRadians(normalizedAngle));
      Logger.recordOutput("RobotController", this.targetPose);
    }

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
