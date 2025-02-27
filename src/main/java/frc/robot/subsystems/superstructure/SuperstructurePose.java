// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final double algaeIntakeAngle = 240.0;
  private static final double groundToCarriageZero = dispenserOrigin2d.getY();
  private static final double tunnelEjectMeters = Units.inchesToMeters(12.0);
  private static final double tunnelEjectMetersReverse = Units.inchesToMeters(10.0);
  // these angles are all measured as a circle with postive angles being CCW and negaitve angles
  // being CW angles
  private static final double L2Angle = 30;
  private static final double L3Angle = 40;
  private static final double L4Angle = 80;
  private static final double L3AngleAlgae = L3Angle - 180.0;
  private static final double L4AngleAlgae = L4Angle + 60.0 - 180.0;

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    START("Start", 0.0, 18.0),
    STOW("Stow", 0.0, 45.0),
    L1("L1", 0.0, 18.0),
    L2("L2", Units.inchesToMeters(3.5), L2Angle + 5),
    L3("L3", Units.inchesToMeters(9.1), L3Angle + 8),
    L4("L4", Units.inchesToMeters(19.2), L4Angle - 10),
    ALGAE_FLOOR_INTAKE(
        "AlgaeFloorIntake",
        Units.inchesToMeters(16.5)
            - groundToCarriageZero
            - (pivotToGripper / 2.0 * Rotation2d.fromDegrees(algaeIntakeAngle).getSin()),
        algaeIntakeAngle),
    ALGAE_L2_INTAKE("AlgaeL2Intake", Units.inchesToMeters(6.5), 165),
    ALGAE_L3_INTAKE("AlgaeL3Intake", Units.inchesToMeters(11.5), 165),
    THROW(() -> elevatorMaxTravel, () -> -40.0),
    PROCESSING("Processing", 0.0, 180),
    ALGAE_STOW("AlgaeStow", 0.0, 100),
    ALGAE_STOW_FRONT("AlgaeStowFront", 0.1, pivotSafeAngle.getDegrees());
    /*
    ,L3_CORAL_REVERSED(
        "L3Reversed",
        DispenserPose.L3_ALGAE.getElevatorHeight(),
        DispenserPose.L3_ALGAE.getDispenserAngleDeg()),
    L4_CORAL_REVERSED(
        "L4Reversed",
        DispenserPose.L4_ALGAE.getElevatorHeight(),
        DispenserPose.L4_ALGAE.getDispenserAngleDeg()),
    L3_CORAL_UNREVERSED("L3Unreversed", DispenserPose.L3_ALGAE.getElevatorHeight(), -40.0),
    L4_CORAL_UNREVERSED("L4Unreversed", DispenserPose.L4_ALGAE.getElevatorHeight(), -40.0);
    */
    private final SuperstructurePose pose;

    private Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromDegrees(pivotAngle.getAsDouble())));
    }

    private Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }
  }

  SuperstructurePose() {
    this(() -> 0.0, Rotation2d::new);
  }
}
