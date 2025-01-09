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
import frc.robot.util.swerve.ControlSettings;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(17.6);
  public static final double odometryFrequency = 150; // Hz
  public static final double trackWidth = Units.inchesToMeters(25);
  public static final double wheelBase = Units.inchesToMeters(25);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation =
      switch (Constants.robotType) {
        case COMPBOT -> Rotation2d.fromRotations(0);
        case SIMBOT -> Rotation2d.fromRotations(0);
        case DEVBOT -> Rotation2d.fromRotations(0.290527);
      };
  public static final Rotation2d frontRightZeroRotation =
      switch (Constants.robotType) {
        case COMPBOT -> Rotation2d.fromRotations(0);
        case SIMBOT -> Rotation2d.fromRotations(0);
        case DEVBOT -> Rotation2d.fromRotations(0.095215)
            .plus(Rotation2d.fromRadians(0.17))
            .minus(Rotation2d.fromRadians(0.151));
      };
  public static final Rotation2d backLeftZeroRotation =
      switch (Constants.robotType) {
        case COMPBOT -> Rotation2d.fromRotations(0);
        case SIMBOT -> Rotation2d.fromRotations(0);
        case DEVBOT -> Rotation2d.fromRotations(0.360352 + 0.25).plus(Rotation2d.fromRadians(0.16));
      };
  public static final Rotation2d backRightZeroRotation =
      switch (Constants.robotType) {
        case COMPBOT -> Rotation2d.fromRotations(0);
        case SIMBOT -> Rotation2d.fromRotations(0);
        case DEVBOT -> Rotation2d.fromRotations(-0.5 + 0.41254)
            .minus(Rotation2d.fromRadians(0.14))
            .plus(Rotation2d.fromRadians(0.076))
            .plus(Rotation2d.fromRadians(0.05));
      };

  // Device CAN IDs
  public static final int pigeonCanId = 0;

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
  public static final double wheelRadiusMeters =
      switch (Constants.robotType) {
        case COMPBOT -> Units.inchesToMeters(3.5);
        case SIMBOT -> Units.inchesToMeters(3.5);
        case DEVBOT -> Units.inchesToMeters(1.98);
      };
  public static final double driveMotorReduction = 6.75; // MK4i L2
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration

  public static final ControlSettings controlSettings =
      switch (Constants.robotType) {
        case COMPBOT -> new ControlSettings(0.001, 0.02, 0.1, 0.11, 0.05, 0.01);
        case SIMBOT -> new ControlSettings(0.05, 0.02, 0.0, 0.0789, 8.0, 0.0);
        case DEVBOT -> new ControlSettings(0.001, 0.02, 0.1, 0.11, 0.3, 0.01);
      };

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

  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final RobotConfig ppConfig =
      switch (Constants.robotType) {
        case COMPBOT -> new RobotConfig(
            74.088,
            6.883,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                1.2,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
        case DEVBOT, SIMBOT -> new RobotConfig(
            74.088,
            6.883,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                1.2,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);
      };
}
