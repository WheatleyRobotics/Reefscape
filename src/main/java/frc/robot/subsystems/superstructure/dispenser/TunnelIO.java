// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.dispenser;

import org.littletonrobotics.junction.AutoLog;

public interface TunnelIO {
  @AutoLog
  static class TunnelIOInputs {
    public boolean talonConnected = false;
    public boolean CANRangeConnected = false;
    public boolean hasCoral = false;
    public double measuredTimestamp = 0.0;
    public double talonPositionRads = 0.0;
    public double talonVelocityRadsPerSec = 0.0;
    public double talonAppliedVoltage = 0.0;
    public double talonSupplyCurrentAmps = 0.0;
    public double talonTorqueCurrentAmps = 0.0;
    public double talonTempCelsius = 0.0;
  }

  default void updateInputs(TunnelIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  /* Run rollers at volts */
  default void runVolts(double volts) {}

  /* Run rollers at velocity using PIDF control */
  default void runVelocity(double velocityRadsPerSec) {}

  /* Stop rollers */
  default void stop() {}

  default void setBrakeMode(boolean enabled) {}

  /* Set PIDF gains for velocity control */
  default void setPID(double kP, double kI, double kD, double kF) {}
}
