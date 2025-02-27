// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.AutoScore;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import lombok.Data;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

@Data
public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  @AutoLogOutput(key = "RobotState/isAuto")
  private boolean isAuto = false;

  @AutoLogOutput(key = "RobotState/Superstructure")
  private SuperstructureState superstructureState = SuperstructureState.START;

  @AutoLogOutput(key = "RobotState/DesiredState")
  private SuperstructureState desiredState = SuperstructureState.START;

  @AutoLogOutput(key = "RobotState/hasDesiredState")
  private boolean hasDesiredState = false;

  @AutoLogOutput(key = "RobotState/Pose")
  private Pose2d pose = new Pose2d();

  @AutoLogOutput(key = "RobotState/Zone")
  private Zones currentZone = Zones.Z1;

  @AutoLogOutput(key = "RobotState/ClearedReef")
  private boolean clearedReef = false;

  @Getter
  @AutoLogOutput(key = "RobotState/shouldTrigSolve")
  private boolean shouldTrigSolve = true;

  @Getter
  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, pose.getRotation());
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  @Getter
  public enum Zones {
    Z1(0),
    Z2(1),
    Z3(2),
    Z4(3),
    Z5(4),
    Z6(5);
    private final int face;

    Zones(int face) {
      this.face = face;
    }
  }

  public void update() {
    updateZone();
    updateIsClearedReef();
  }

  public void resetPose(Pose2d pose) {
    this.pose = pose;
  }

  public void updateZone() {
    Translation2d flippedReef;
    double angle;
    if (AllianceFlipUtil.shouldFlip()) {
      flippedReef = AllianceFlipUtil.apply(FieldConstants.Reef.center);
      angle = -Math.atan2(pose.getY() - flippedReef.getY(), pose.getX() - flippedReef.getX());

      angle += Math.PI / 6.0;

    } else {
      flippedReef = FieldConstants.Reef.center;
      angle = -Math.atan2(pose.getY() - flippedReef.getY(), pose.getX() - flippedReef.getX());

      angle += Math.PI + Math.PI / 6.0;
    }
    double normalizedAngle = (angle + (2 * Math.PI)) % (2 * Math.PI);

    if (normalizedAngle >= 0 && normalizedAngle < Math.PI / 3) {
      currentZone = Zones.Z1;
    } else if (normalizedAngle >= Math.PI / 3 && normalizedAngle < 2 * Math.PI / 3) {
      currentZone = Zones.Z2;
    } else if (normalizedAngle >= 2 * Math.PI / 3 && normalizedAngle < Math.PI) {
      currentZone = Zones.Z3;
    } else if (normalizedAngle >= Math.PI && normalizedAngle < 4 * Math.PI / 3) {
      currentZone = Zones.Z4;
    } else if (normalizedAngle >= 4 * Math.PI / 3 && normalizedAngle < 5 * Math.PI / 3) {
      currentZone = Zones.Z5;
    } else if (normalizedAngle >= 5 * Math.PI / 3 && normalizedAngle < 2 * Math.PI) {
      currentZone = Zones.Z6;
    }
  }

  public void updateIsClearedReef() {
    double distanceToLeft =
        FieldConstants.getBranch(currentZone, false)
            .getTranslation()
            .getDistance(pose.getTranslation());
    double distanceToRight =
        FieldConstants.getBranch(currentZone, true)
            .getTranslation()
            .getDistance(pose.getTranslation());
    clearedReef =
        !(distanceToLeft < AutoScore.minClearReefDistance.get())
            && !(distanceToRight < AutoScore.minClearReefDistance.get());
  }

  public void setDesiredState(SuperstructureState state) {
    desiredState = state;
    hasDesiredState = true;
  }
}
