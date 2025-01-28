package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double positionRads = 0.0;
    public double absoluteEncoderPositionMeters = 0.0;
    public double relativeEncoderPositionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  default void runSetpoint(double setpointRads, double feedforward) {}

  /** Run motors at volts */
  default void runVolts(double volts) {}

  /** Run motors at current */
  default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  default void setPID(double p, double i, double d) {}

  /** Stops motors */
  default void stop() {}
}
