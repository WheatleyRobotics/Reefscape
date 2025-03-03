// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
  public static final double odometryFrequency = 100; // Hz
  public static final double trackWidth = Units.inchesToMeters(28);
  public static final double wheelBase = Units.inchesToMeters(28);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(19.3);
  public static final double maxAngularSpeed = Units.feetToMeters(19.3) / driveBaseRadius;
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Rotation2d();
        case COMPBOT -> Rotation2d.fromRotations(0.302734); // 0.31
        case DEVBOT -> Rotation2d.fromRotations(0.290527); // 0.290527
      };
  public static final Rotation2d frontRightZeroRotation =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Rotation2d();
        case COMPBOT -> Rotation2d.fromRotations(0.197021); // 0.22
        case DEVBOT -> Rotation2d.fromRotations(0.095215)
            .plus(Rotation2d.fromRadians(0.17))
            .minus(Rotation2d.fromRadians(0.151)); // 0.095215
      };
  public static final Rotation2d backLeftZeroRotation =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Rotation2d();
        case COMPBOT -> Rotation2d.fromRotations(-0.106201); // -0.094
        case DEVBOT -> Rotation2d.fromRotations(0.360352 + 0.25)
            .plus(Rotation2d.fromRadians(0.16)); // 0.360352
      };

  public static final Rotation2d backRightZeroRotation =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Rotation2d();
        case COMPBOT -> Rotation2d.fromRotations(-0.098145); // 0.096
        case DEVBOT -> Rotation2d.fromRotations(-0.5 + 0.41254)
            .minus(Rotation2d.fromRadians(0.14))
            .plus(Rotation2d.fromRadians(0.076))
            .plus(Rotation2d.fromRadians(0.05)); // -0.5 + 0.41254
      };

  // Device CAN IDs
  public static final int pigeonCanId = 0;

  // Front left 1
  // Front right 2
  // Back left 3
  // Back right 4

  public static final int frontLeftDriveCanId = 41;
  public static final int frontRightDriveCanId = 42;
  public static final int backLeftDriveCanId = 43;
  public static final int backRightDriveCanId = 44;

  public static final int frontLeftTurnCanId = 51;
  public static final int frontRightTurnCanId = 52;
  public static final int backLeftTurnCanId = 53;
  public static final int backRightTurnCanId = 54;

  public static final int frontLeftTurnCancoderId = 31;
  public static final int frontRightTurnCancoderId = 32;
  public static final int backLeftTurnCancoderId = 33;
  public static final int backRightTurnCancoderId = 34;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.935);

  public static final double driveMotorReduction = 6.12; // MK4i L3
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0023;
  public static final double driveKd = 0.04;
  public static final double driveKs = 0.11;
  public static final double driveKv = 0.10;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150.0 / 7;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.6;
  public static final double turnKd = 0.04;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 58.967;
  public static final double robotMOI = 4.75;
  public static final double wheelCOF = 1.5;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
