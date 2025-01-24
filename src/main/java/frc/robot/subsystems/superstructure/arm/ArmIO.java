// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean armMotorConnected = true;

    public double positionRads = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double relativeEncoderPositionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelcius = 0.0;
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(ArmIOInputs inputs) {}

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
