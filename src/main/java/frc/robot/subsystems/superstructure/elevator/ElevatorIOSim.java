// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;
  private double appliedVoltage = 0.0;
  private final PIDController currentController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);

  private final double drumRadiusMeters;

  /**
   * Creates a new GenericSlamElevator Sim implementation
   *
   * @param maxLengthMeters Position in motor radians from top to bottom of slam elevator
   */
  public ElevatorIOSim(double maxLengthMeters, double reduction, double drumRadiusMeters) {
    sim =
        new ElevatorSim(
            DCMotor.getFalcon500(2),
            reduction,
            0.5,
            drumRadiusMeters,
            0.0,
            maxLengthMeters,
            false,
            0.0);
    sim.setState(maxLengthMeters / 2.0, 0);
    this.drumRadiusMeters = drumRadiusMeters;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.positionRads = sim.getPositionMeters() / drumRadiusMeters;
    inputs.velocityRadsPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void runVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }
}
