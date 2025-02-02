// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Pose")
  private Pose2d pose = new Pose2d();

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Zone")
  private Zones currentZone = Zones.Z1;

  public enum Zones {
    Z1(0),
    Z2(1),
    Z3(2),
    Z4(3),
    Z5(4),
    Z6(5);
    @Getter private final int face;

    Zones(int face) {
      this.face = face;
    }
  }

  public void update() {
    Translation2d flippedReef;
    if (AllianceFlipUtil.shouldFlip()) {
      flippedReef = AllianceFlipUtil.apply(FieldConstants.Reef.center);
    } else {
      flippedReef = FieldConstants.Reef.center;
    }
    double angle = -Math.atan2(pose.getY() - flippedReef.getY(), pose.getX() - flippedReef.getX());

    angle += Math.PI / 6.0;

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

  public void resetPose(Pose2d pose) {
    this.pose = pose;
  }
}
