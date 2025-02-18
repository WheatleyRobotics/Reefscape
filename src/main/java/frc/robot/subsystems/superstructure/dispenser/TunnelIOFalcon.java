// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.dispenser;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class TunnelIOFalcon implements TunnelIO {
  private final TalonFX talon;
  // private final CANrange canRange;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // private final StatusSignal<Distance> distance;
  // private final StatusSignal<Boolean> isDetected;

  // Single shot for voltage mode, robot loop will call continuously
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final double reduction = 0;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public TunnelIOFalcon() {
    talon = new TalonFX(5);
    // canRange = new CANrange(4);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 40;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
    proximityParamsConfigs.withProximityHysteresis(2);
    proximityParamsConfigs.withProximityThreshold(10);
    canRangeConfig.withProximityParams(proximityParamsConfigs);
    /*
    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig));

    distance = canRange.getDistance();
    isDetected = canRange.getIsDetected();

     */

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius // ,distance,isDetected
                ));
    ParentDevice.optimizeBusUtilizationForAll( // canRange,
        talon);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
    inputs.talonConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK());
    inputs.CANRangeConnected = false;
    // connectedDebouncer.calculate(BaseStatusSignal.refreshAll(distance, isDetected).isOK());
    inputs.hasCoral = false; // isDetected.getValue();
    inputs.talonPositionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.talonVelocityRadsPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.talonAppliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.talonSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.talonTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.talonTempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runTorqueCurrent(double current) {
    talon.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () ->
                tryUntilOk(
                    5,
                    () ->
                        talon.setNeutralMode(
                            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)))
        .start();
  }
}
