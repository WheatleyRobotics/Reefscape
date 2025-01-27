package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.Constants.RobotType.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final double reduction = 1.0; // TODO: Tune everything
  public static int leadMotorID =
      switch (Constants.getRobotType()) {
        case SIMBOT -> 11;
        case DEVBOT -> 25;
        case COMPBOT -> 11;
      };

  public static int followerMotorID =
      switch (Constants.getRobotType()) {
        case SIMBOT -> 11;
        case DEVBOT -> 25;
        case COMPBOT -> 11;
      };

  public static final Rotation2d minAngle =
      switch (Constants.getRobotType()) {
        default -> Rotation2d.fromDegrees(5.8); // Measured from hardstop 3/31/24
        case DEVBOT -> Rotation2d.fromDegrees(10.0);
      };
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(110.0);

  public static final boolean leaderInverted = false;

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
