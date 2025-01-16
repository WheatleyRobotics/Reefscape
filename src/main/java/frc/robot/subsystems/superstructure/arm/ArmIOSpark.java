// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.armEncoderPostionFactor;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.armEncoderVelocityFactor;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.armKd;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.armKp;
import static frc.robot.subsystems.superstructure.arm.ArmConstants.armMotorCurrentLimit;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {
  private final SparkBase armSpark;
  private final RelativeEncoder armEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController armController;
  private final Debouncer armConnectedDebounce = new Debouncer(0.5);

  /** Creates a new ArmIOSpark. */
  public ArmIOSpark() {
    armSpark = new SparkFlex(ArmConstants.motorID, MotorType.kBrushless);
    armEncoder = armSpark.getEncoder();
    armController = armSpark.getClosedLoopController();
    absoluteEncoder = armSpark.getAbsoluteEncoder();
    var config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(armMotorCurrentLimit)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(armEncoderPostionFactor)
        .velocityConversionFactor(armEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(armKp, 0.0, armKd, 0.0);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        armSpark,
        5,
        () ->
            armSpark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // intialize encoder using through bore value
    tryUntilOk(
        armSpark,
        5,
        () -> armEncoder.setPosition(Units.rotationsToRadians(absoluteEncoder.getPosition())));
  }

  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(armSpark, armEncoder::getPosition, (value) -> inputs.relativeEncoderPositionRads = value);
    ifOk(armSpark, armEncoder::getVelocity, (value) -> inputs.velocityRadsPerSec = value);
    ifOk(armSpark, armSpark::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    ifOk(
        armSpark,
        new DoubleSupplier[] {armSpark::getAppliedOutput, armSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(armSpark, armSpark::getMotorTemperature, (value) -> inputs.tempCelcius = value);
    inputs.motorConnected = armConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    armController.setReference(
        Units.radiansToRotations(setpointRads),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void runVolts(double volts) {
    armController.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void runCurrent(double amps) {
    armController.setReference(amps, ControlType.kCurrent);
  }

  @Override
  public void stop() {
    armSpark.set(0);
  }
}
