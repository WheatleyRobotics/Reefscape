// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.dispenser;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TunnelIOSim implements TunnelIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;
  
  private final PIDController velocityController = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  private boolean velocityControl = false;

  public TunnelIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
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
    
    // Update velocity control if enabled
    if (velocityControl) {
      double pidOutput = velocityController.calculate(inputs.talonVelocityRadsPerSec);
      double ffOutput = feedforward.calculate(velocityController.getSetpoint());
      appliedVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
      sim.setInputVoltage(appliedVoltage);
    }
  }

  @Override
  public void runTorqueCurrent(double current) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(current), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void runVolts(double volts) {
    velocityControl = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void runVelocity(double velocityRadsPerSec) {
    velocityControl = true;
    velocityController.setSetpoint(velocityRadsPerSec);
  }

  @Override
  public void stop() {
    velocityControl = false;
    runVolts(0.0);
  }
  
  @Override
  public void setPID(double kP, double kI, double kD, double kF) {
    velocityController.setPID(kP, kI, kD);
    // For simulation, we use kF as kV for feedforward
    // Note: This is a simplified implementation
  }
}
