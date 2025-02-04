// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class SuperstructureConstants {
  // 2d position of superstructure root on robot (x forward from back, y off the bellypan)
  public static final Translation2d superstructureOrigin2d =
      new Translation2d(0.10686, 0.06357); // TODO Find all of these constants in the CAD
  public static final Translation3d superstructureOrigin3d =
      new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());
  public static final double pivotLength = 0.16;
  public static final double pivotLengthBack = 0.05;
  public static final double pivotToGripper = 0.207;
  public static final double G = 9.807;
  // From inside face to inside face
  public static final double stageHeight = 0.808; // Measured from CAD
  public static final double stageThickness = 0.025;
  public static final double carriageToStage = 0.12;
  public static final double stageToStage = 0.127;
  public static final double elevatorHeightMeters =
      -(stageThickness * 2 + carriageToStage)
          + 2 * (stageHeight - stageToStage)
          + stageHeight
          - carriageToStage;
  public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90);
  public static final Translation3d carriageOrigin3d =
      superstructureOrigin3d.plus(
          new Translation3d(
              stageThickness * 2 + carriageToStage,
              new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0)));

  public static final Rotation2d pivotSafeAngle = Rotation2d.fromDegrees(-40.0);
}
