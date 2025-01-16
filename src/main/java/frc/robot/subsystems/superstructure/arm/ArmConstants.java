// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstants {
  // reduction is 12:62 18:60 12:65 1:15, 18:60
  public static final double reduction = (15 / 1) * (60.0 / 18.0);
  public static final Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static final Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static final Rotation2d minAngle =
      switch (Constants.getRobotType()) {
        default -> Rotation2d.fromDegrees(5.8); // Measured from hardstop 3/31/24
        case DEVBOT -> Rotation2d.fromDegrees(10.0);
      };
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);
  public static final int armMotorCurrentLimit = 30;
  public static final double armEncoderPostionFactor =
      2 * Math.PI / reduction; // Rotor Rotations -> Wheel Radians
  public static final double armEncoderVelocityFactor =
      (2 * Math.PI) / 60 / reduction; // Rotor RPM -> Wheel Rad/Sec
  public static final double armKp = 0.1;
  public static final double armKd = 0.1;
  public static final int motorID =
      switch (Constants.getRobotType()) {
        default -> 11;
        case DEVBOT -> 25;
      };
  public static final int armEncoderID =
      switch (Constants.getRobotType()) {
        default -> 0;
        case DEVBOT -> 42;
      };

  public static final boolean leaderInverted = false;

  /** The offset of the arm encoder in radians. */
  public static final double armEncoderOffsetRads =
      switch (Constants.getRobotType()) {
        default -> 0; // 1.21784482 todo find encodedr offset
          // corresponding to an arm position of 0.1043106935762236 rad
        case DEVBOT -> 0; // -1.233 - Math.PI / 2.0;
      };

  public static final double armLength =
      switch (Constants.getRobotType()) {
        case DEVBOT -> Units.inchesToMeters(20); // need to measure our arm
        default -> Units.inchesToMeters(20); // same
      };

  public static final Gains gains =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case DEVBOT -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
      };

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 15);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
