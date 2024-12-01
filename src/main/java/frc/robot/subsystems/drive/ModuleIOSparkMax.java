// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.Fault;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO or NEO 550), and
 * analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware configurations
 * (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward motion on the drive motor
 * will propel the robot forward) and copy the reported values from the absolute encoders using AdvantageScope. These
 * values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
    // Gear ratios for SDS MK4i L3, adjust as necessary
    private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final CANSparkFlex driveSparkFlex;
    private final CANSparkMax turnSparkMax;
    private final CANcoder cancoder;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnRelativeEncoder;
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnAbsolutePosition;
    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;
    private final PIDController driveController;
    private final PIDController turnController;

    public ModuleIOSparkMax(int index) {
        switch (index) {
            case 0: // FL
                driveSparkFlex = new CANSparkFlex(41, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(51, MotorType.kBrushless);
                cancoder = new CANcoder(31);
                absoluteEncoderOffset = Rotation2d.fromRotations(0.290527);
                break;
            case 1: // FR
                driveSparkFlex = new CANSparkFlex(42, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(52, MotorType.kBrushless);
                cancoder = new CANcoder(32);
                absoluteEncoderOffset = Rotation2d.fromRotations(0.095215);
                break;
            case 2: // BL
                driveSparkFlex = new CANSparkFlex(43, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(53, MotorType.kBrushless);
                cancoder = new CANcoder(33);
                absoluteEncoderOffset = Rotation2d.fromRotations(-0.360352);
                break;
            case 3: // BR
                driveSparkFlex = new CANSparkFlex(44, MotorType.kBrushless);
                turnSparkMax = new CANSparkMax(54, MotorType.kBrushless);
                cancoder = new CANcoder(34);
                absoluteEncoderOffset = Rotation2d.fromRotations(-0.5 + 0.41254);
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        driveSparkFlex.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkFlex.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        driveEncoder = driveSparkFlex.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        turnSparkMax.setInverted(isTurnMotorInverted);
        driveSparkFlex.setSmartCurrentLimit(55);
        turnSparkMax.setSmartCurrentLimit(40);
        driveSparkFlex.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        driveSparkFlex.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
        timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            double value = driveEncoder.getPosition();
            if (driveSparkFlex.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
            } else {
                return OptionalDouble.empty();
            }
        });
        turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            double value = turnRelativeEncoder.getPosition();
            if (driveSparkFlex.getLastError() == REVLibError.kOk) {
                return OptionalDouble.of(value);
            } else {
                return OptionalDouble.empty();
            }
        });

        driveSparkFlex.burnFlash();
        turnSparkMax.burnFlash();
        cancoder.getConfigurator().apply(new CANcoderConfiguration());
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

        driveController = new PIDController(0, 0.0, 0);
        turnController = new PIDController(0, 0.0, 0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(turnAbsolutePosition);
        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity()) / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
                .toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSparkFlex.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public List<Fault> selfCheck() {
        List<Fault> faults = new ArrayList<>();
        faults.addAll(Fault.checkForFaults("CANcoder " + cancoder.getDeviceID(), cancoder));
        faults.addAll(Fault.checkForFaults("Drive SparkFlex " + driveSparkFlex.getDeviceId(), driveSparkFlex));
        faults.addAll(Fault.checkForFaults("Turn SparkMax " + turnSparkMax.getDeviceId(), turnSparkMax));
        return faults;
    }
}
