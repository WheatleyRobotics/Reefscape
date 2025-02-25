package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  class ClimbIOInputs {
    public boolean ClimbConncected = true;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
    public boolean motorConnected;
  }

  default void updateInputs(ClimbIOInputs climb) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}
}
