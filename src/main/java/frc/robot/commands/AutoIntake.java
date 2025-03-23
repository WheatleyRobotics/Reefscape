package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.IntStream;

public class AutoIntake extends Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("AutoIntake/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("AutoIntake/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoIntake/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoIntake/ThetakD");
  private static final LoggedTunableNumber txnc_offset =  new LoggedTunableNumber("AutoIntake/TXNCOffset");
  private static final LoggedTunableNumber max_velocity =  new LoggedTunableNumber("AutoIntake/MaxVelocity");
  private static final LoggedTunableNumber max_omega =  new LoggedTunableNumber("AutoIntake/MaxOmega");

  private final Drive drive;
  private Rotation2d targetSourceAngle;

  private final PIDController driveController = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
  private final PIDController thetaController = new PIDController(0, 0, 0, Constants.loopPeriodSecs);

  static{
    drivekP.initDefault(2.1);
    drivekD.initDefault(0.2);
    thetakP.initDefault(3.0);
    thetakD.initDefault(0.1);
    txnc_offset.initDefault(1.0);
    max_velocity.initDefault(1.0);
  }

  public AutoIntake(Drive drive) {
    this.drive = drive;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", 1);
    // Determine which coral station to target
    Pose2d currentPose = RobotState.getInstance().getPose();
    double distToRight = currentPose.getTranslation()
            .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation());
    double distToLeft = currentPose.getTranslation()
            .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation());

    targetSourceAngle = (distToRight < distToLeft)
            ? FieldConstants.CoralStation.rightCenterFace.getRotation()
            : FieldConstants.CoralStation.leftCenterFace.getRotation();

    driveController.setP(drivekP.get());
    driveController.setD(drivekD.get());
    thetaController.setP(thetakP.get());
    thetaController.setD(thetakD.get());

    driveController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    // Update PID values if they've changed
    if(drivekP.hasChanged(hashCode())
            || drivekD.hasChanged(hashCode())
            || thetakP.hasChanged(hashCode())
            || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
    }

    LimelightHelpers.RawDetection[] rawDetections = LimelightHelpers.getRawDetections("limelight");
    Logger.recordOutput("AutoAlign/DetectionCount", rawDetections.length);

    if (rawDetections.length > 0) {
      int index = 0;
      double greatestTa = 0.0;
      for(int i = 0; i < rawDetections.length; i++) {
        if(rawDetections[i].classId == 1){
          if(rawDetections[i].ta >  greatestTa){
            greatestTa = rawDetections[i].ta;
            index = i;
          }
        }
      }

      double txnc = rawDetections[index].txnc;

      // Log detection information
      Logger.recordOutput("AutoAlign/TXNC", txnc);
      Logger.recordOutput("AutoAlign/GreatestTA", greatestTa);
      Logger.recordOutput("AutoAlign/DetectionID", index);

      // Calculate feedback for driving and rotation
      double xFeedback = driveController.calculate(txnc, txnc_offset.get());
      double thetaFeedback = thetaController.calculate(
              RobotState.getInstance().getPose().getRotation().getRadians(),
              targetSourceAngle.getRadians()
      );

      xFeedback = MathUtil.clamp(xFeedback, -max_velocity.get(), max_velocity.get());
      thetaFeedback = MathUtil.clamp(thetaFeedback, -max_omega.get(), max_omega.get());

      drive.runVelocity(new ChassisSpeeds(xFeedback, 0.0, thetaFeedback));
    } else {
      drive.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return RobotState.getInstance().isHasCoral();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  public static Command getAutoIntake(Drive drive, Superstructure superstructure) {
    return
            Commands.sequence(
                    new DriveToPose(
                            drive,
                            () -> FieldConstants.addOffset(RobotState.getInstance()
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(
                                            FieldConstants.CoralStation.rightCenterFace.getTranslation())
                                    < RobotState.getInstance()
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(
                                            FieldConstants.CoralStation.leftCenterFace.getTranslation())
                                    ? FieldConstants.CoralStation.rightCenterFace
                                    : FieldConstants.CoralStation.leftCenterFace, 0.1)
                    )
                            .until(() -> LimelightHelpers.getDetectorClass("limelight").equals("coral")),
                    new AutoIntake(drive).deadlineFor(
                            superstructure.runGoal(SuperstructureState.INTAKE))
    );
  }
}