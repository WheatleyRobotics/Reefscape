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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "leftCamera";
  public static String camera1Name = "rightCamera";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 = // Left Camera
      new Transform3d(
          Units.inchesToMeters(9.21),
          Units.inchesToMeters(11.431),
          Units.inchesToMeters(7.565), // 6.065 + 1.5
          new Rotation3d(
              0,
              Units.degreesToRadians(-15),
              Units.degreesToRadians(-15))); // pitch of -15 and yaw of -15
  public static Transform3d robotToCamera1 = // Right Camera
      new Transform3d(
          Units.inchesToMeters(9.21), // 11.431
          Units.inchesToMeters(-11.431), // 9.21
          Units.inchesToMeters(7.565),
          new Rotation3d(
              0,
              Units.degreesToRadians(-15),
              Units.degreesToRadians(15))); // pitch of -15 and yaw of 15

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.1;

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatagFactor = 4; // More stable than full 3D solve
  public static double linearStdDevPhotonMultiTag = 0.8;
  public static double linearStdDevPhotonTrig = 0.25;
  public static double linearStdDevPhotonSingleTag = 2.0;
}
