package frc.robot.subsystems.superstructure.arm;

import static frc.robot.Constants.RobotType.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ArmConstants {
  public static final double reduction = 1.0; // TODO: Tune everything
  public static int motorID =
      switch (Constants.getRobotType()) {
        case SIMBOT -> 11;
        case DEVBOT -> 25;
        case COMPBOT -> 11;
      };

  public static int encoderID =
      switch (Constants.getRobotType()) {
        case SIMBOT -> 11;
        case DEVBOT -> 25;
        case COMPBOT -> 11;
      };

  public static final double armEncoderOffsetRads =
      switch (Constants.getRobotType()) {
        default -> 1.21784482;
          // corresponding to an arm position of 0.1043106935762236 rad
        case DEVBOT -> -1.233 - Math.PI / 2.0;
      };

  public static final Rotation2d minAngle =
      switch (Constants.getRobotType()) {
        default -> Rotation2d.fromDegrees(5.8); // Measured from hardstop 3/31/24
        case DEVBOT -> Rotation2d.fromDegrees(10.0);
      };
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);

  public static final double armLength = 5;

  public static final Translation2d armOrigin = new Translation2d(-0.238, 0.298);

  public static final boolean leaderInverted = false;

  public static final Gains gains =
      switch (Constants.getRobotType()) {
        case SIMBOT -> new Gains(90, 00, 0.0, 0.0, 0.0, 0.0, 0.0);
        case DEVBOT -> new Gains(.05, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(.05, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
      };
  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 15);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
