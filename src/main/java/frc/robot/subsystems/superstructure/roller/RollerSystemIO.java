// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public boolean talonConnected = false;
    public boolean CANRangeConnected = false;
    public boolean hasCoral = false;
    public double talonPositionRads = 0.0;
    public double talonVelocityRadsPerSec = 0.0;
    public double talonAppliedVoltage = 0.0;
    public double talonSupplyCurrentAmps = 0.0;
    public double talonTorqueCurrentAmps = 0.0;
    public double talonTempCelsius = 0.0;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  /* Run rollers at volts */
  default void runVolts(double volts) {}

  /* Stop rollers */
  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
