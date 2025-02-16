// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;

  public RollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    inputs.talonConnected = true;
    inputs.CANRangeConnected = true;
    sim.update(Constants.loopPeriodSecs);
    inputs.talonPositionRads = sim.getAngularPositionRad();
    inputs.talonVelocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.talonAppliedVoltage = appliedVoltage;
    inputs.talonSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.talonTorqueCurrentAmps =
        gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
  }

  @Override
  public void runTorqueCurrent(double current) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(current), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
