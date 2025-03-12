// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class SuperstructureConstants {
  public static final double pivotLength = 0.16;
  public static final double pivotLengthBack = 0.05;
  public static final double pivotToGripper = 0.207;
  public static final double G = 9.807;
  // From inside face to inside face, measured from CAD
  public static final double stageHeight = Units.inchesToMeters(39);
  public static final double stageToStageOffset = Units.inchesToMeters(0.175);
  public static final double stageThickness = Units.inchesToMeters(1.0);
  public static final double dispenserToCarriage = Units.inchesToMeters(4.0);
  public static final double stageToStage = Units.inchesToMeters(5.0);
  // Height from superstructure origin to bottom face of first stage at maxed height
  // minus the difference from superstructure origin to dispenser origin and from the topped out
  // position to the dispenser
  public static final double elevatorMaxTravel = Units.inchesToMeters(19);

  // 2d position of both superstructure and dispenser origin on robot (x forward from center, y off
  // the ground)
  public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90);
  public static final Translation2d superstructureOrigin2d = new Translation2d(0.10686, 0.06357);
  public static final Translation3d superstructureOrigin3d =
      new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());
  public static final Translation2d dispenserOrigin2d =
      superstructureOrigin2d.plus(
          new Translation2d(
              (stageThickness + stageToStageOffset) * 3 + dispenserToCarriage, elevatorAngle));
  public static final Translation3d dispenserOrigin3d =
      new Translation3d(dispenserOrigin2d.getX(), 0.0, dispenserOrigin2d.getY());

  public static final Rotation2d pivotSafeAngle = Rotation2d.fromDegrees(-40.0);
  public static final LoggedTunableNumber throwHeight =
      new LoggedTunableNumber("Superstructure/Throw/Height", elevatorMaxTravel);
  public static final LoggedTunableNumber throwVelocity =
      new LoggedTunableNumber("Superstructure/Throw/Velocity", 3.0);
}
