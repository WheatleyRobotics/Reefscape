package frc.robot.util.swerve;

import lombok.Getter;

@Getter
public class ControlSettings {
  public double driveKp;
  public double driveKd;
  public double driveKs;
  public double driveKv;
  public double turnKp;
  public double turnKd;

  public ControlSettings(
      double driveKp,
      double driveKd,
      double driveKs,
      double driveKv,
      double turnKp,
      double turnKd) {
    this.driveKp = driveKp;
    this.driveKd = driveKd;
    this.driveKs = driveKs;
    this.driveKv = driveKv;
    this.turnKp = turnKp;
    this.turnKd = turnKd;
  }
}
