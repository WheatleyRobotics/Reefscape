package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class ArmIOFalcon implements ArmIO {
  private final TalonFX armMotor;
  private final CANcoder armEncoder;

  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<Angle> encoderAbsolutePositionRotations;
  private final StatusSignal<Angle> encoderRelativePositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  //TODO: Remove FOC control
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ArmIOFalcon() {
    armMotor = new TalonFX(ArmConstants.motorID);
    armEncoder = new CANcoder(ArmConstants.encoderID);
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(ArmConstants.armEncoderOffsetRads);
    armEncoder.getConfigurator().apply(armEncoderConfig, 1.0);

    // Leader motor configs
    config.Slot0.kP = ArmConstants.gains.kP();
    config.Slot0.kI = ArmConstants.gains.kI();
    config.Slot0.kD = ArmConstants.gains.kD();
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.Inverted =
        ArmConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = ArmConstants.encoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = ArmConstants.reduction;
    config.Feedback.SensorToMechanismRatio = 1.0;
    armMotor.getConfigurator().apply(config, 1.0);

    // Status signals
    internalPositionRotations = armMotor.getPosition();
    encoderAbsolutePositionRotations = armEncoder.getAbsolutePosition();
    encoderRelativePositionRotations = armEncoder.getPosition();
    velocityRps = armMotor.getVelocity();
    appliedVoltage = armMotor.getMotorVoltage();
    supplyCurrent = armMotor.getSupplyCurrent();
    torqueCurrent = armMotor.getTorqueCurrent();
    tempCelsius = armMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    BaseStatusSignal.setUpdateFrequencyForAll(
        500, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

    armMotor.optimizeBusUtilization();
    armEncoder.optimizeBusUtilization();
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(
                encoderAbsolutePositionRotations, encoderRelativePositionRotations)
            .isOK();

    inputs.positionRads =
        Units.rotationsToRadians(
            internalPositionRotations.getValueAsDouble()); // TODO: What unit is this in?
    inputs.absoluteEncoderPositionRads =
        Units.rotationsToRadians(encoderAbsolutePositionRotations.getValueAsDouble())
            - ArmConstants.armEncoderOffsetRads; // Negate internal offset
    inputs.relativeEncoderPositionRads =
        Units.rotationsToRadians(encoderRelativePositionRotations.getValueAsDouble())
            - ArmConstants.armEncoderOffsetRads;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRps.getValueAsDouble());
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    armMotor.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward));
  }

  @Override
  public void runVolts(double volts) {
    armMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    armMotor.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    armMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    armMotor.getConfigurator().apply(config, 0.01);
  }

  @Override
  public void stop() {
    armMotor.setControl(new NeutralOut());
  }
}
