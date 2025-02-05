// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.superstructure.slam.Slam;
import frc.robot.util.EqualsUtil;
import frc.robot.util.RobotState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private static final Translation3d intakeOrigin3d = new Translation3d(0.2850, 0.0, 0.1630);
  private static final Rotation2d ambiguousIntakePosition =
      Slam.minAngle.interpolate(Slam.maxAngle, 0.5);

  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismLigament2d pivotMechanism;

  public SuperstructureVisualizer(String name) {
    this.name = name;
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            name + " Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY());
    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator",
                Units.inchesToMeters(26.0),
                elevatorAngle.getDegrees(),
                4.0,
                new Color8Bit(Color.kFirstBlue)));
    pivotMechanism =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Pivot",
                Units.inchesToMeters(6.0),
                0.0,
                8.0,
                new Color8Bit(Color.kFirstRed)));
  }

  public void update(
      double elevatorHeightMeters,
      Rotation2d pivotFinalAngle,
      boolean slammed,
      boolean retracting,
      boolean hasAlgae) {
    elevatorMechanism.setLength(
        EqualsUtil.epsilonEquals(elevatorHeightMeters, 0.0)
            ? Units.inchesToMeters(1.0)
            : elevatorHeightMeters);
    pivotMechanism.setAngle(pivotFinalAngle.minus(elevatorAngle));
    Logger.recordOutput("Mechanism2d/" + name, mechanism);

    // Max of top of carriage or starting height
    final double heightFromBottom =
        elevatorHeightMeters + stageThickness * 2.0 + dispenserToCarriage;
    final double firstStageHeight =
        Math.max(
            heightFromBottom + dispenserToCarriage - stageHeight - (stageThickness / 2.0),
            stageThickness * (3.0 / 2.0));
    final double secondStageHeight =
        Math.max(
            firstStageHeight - (stageThickness / 2.0) - stageHeight + stageToStage,
            stageThickness * (1.0 / 2.0) + stageToStageOffset);

    Pose3d pivotPose3d =
        new Pose3d(
            dispenserOrigin3d.plus(
                new Translation3d(
                    elevatorHeightMeters, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            new Rotation3d(
                0.0,
                // Have to invert angle due to CAD??
                Rotation2d.kPi.minus(pivotFinalAngle).plus(elevatorAngle).getRadians(),
                0.0));

    Logger.recordOutput(
        "Mechanism3d/" + name + "/Elevator",
        // Outer Stage
        new Pose3d(
            superstructureOrigin3d.plus(
                new Translation3d(
                    secondStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            new Rotation3d()),
        // Carriage always at top of inner stage
        new Pose3d(
            superstructureOrigin3d.plus(
                new Translation3d(
                    firstStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            new Rotation3d()),
        pivotPose3d);
    Logger.recordOutput(
        "Mechanism3d/" + name + "/AlgaeIntake",
        new Pose3d(
            intakeOrigin3d,
            new Rotation3d(
                0.0,
                Rotation2d.kPi
                        .minus(
                            (slammed
                                ? (retracting ? Slam.maxAngle : Slam.minAngle)
                                : ambiguousIntakePosition))
                        .getRadians()
                    - Math.PI / 2.0,
                0.0)));
    if (hasAlgae) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Algae",
          new Pose3d(RobotState.getInstance().getEstimatedPose())
              .transformBy(new Transform3d(Pose3d.kZero, pivotPose3d))
              .transformBy(
                  new Transform3d(
                      pivotToGripper + Units.inchesToMeters(4.0), 0.0, -0.1, Rotation3d.kZero))
              .getTranslation());
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Algae", new Translation3d[] {});
    }
  }

  public static void updateSimIntake(double angleRad) {
    Logger.recordOutput(
        "Mechanism3d/AlgaeIntakeSim",
        new Pose3d(intakeOrigin3d, new Rotation3d(0.0, Math.PI / 2.0 - angleRad, 0.0)));
  }
}
