// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber smoothVelocity =
      new LoggedTunableNumber("Elevator/SmoothVelocity", profileConstraints.maxVelocity * 0.75);
  private static final LoggedTunableNumber smoothAcceleration =
      new LoggedTunableNumber(
          "Elevator/SmoothAcceleration", profileConstraints.maxAcceleration * 0.5);
  private static final LoggedTunableNumber prepareClimbVelocity =
      new LoggedTunableNumber("Elevator/PrepareClimbVelocity", 1.5);
  private static final LoggedTunableNumber prepareClimbAcceleration =
      new LoggedTunableNumber("Elevator/PrepareClimbAcceleration", 2.5);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Elevator/LowerLimitDegrees", minHeight);
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Elevator/UpperLimitDegrees", maxheight);
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Elevator/PartialStowUpperLimitDegrees", 30.0);

  // Profile constraints
  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> smoothProfileConstraints =
      () -> new TrapezoidProfile.Constraints(smoothVelocity.get(), smoothAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> prepareClimbProfileConstraints =
      () ->
          new TrapezoidProfile.Constraints(
              prepareClimbVelocity.get(), prepareClimbAcceleration.get());

  @RequiredArgsConstructor
  public enum Goal {
    STOW(() -> 0),
    L1(new LoggedTunableNumber("Arm/L1", 40.0)),
    L2(new LoggedTunableNumber("Arm/L2", 45.0)),
    L3(new LoggedTunableNumber("Arm/L3", 110.0)),
    L4(new LoggedTunableNumber("Arm/L4", 55.0));

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(elevatorSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput @Setter private double currentCompensation = 0.0;
  private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalHeight;
  private ElevatorFeedforward ff;

  private final Alert leaderMotorDisconnected =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerMotorDisconnected =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  public void setOverrides(
      BooleanSupplier disableOverride,
      BooleanSupplier coastOverride,
      BooleanSupplier halfStowOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    coastSupplier = coastOverride;
  }

  private double getStowHeight() {
    return minHeight;
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    // Check if disabled
    // Also run first cycle of auto to reset elevator
    if (disableSupplier.getAsBoolean()
        || (Constants.getMode() == Constants.Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto)) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
    }
    // Track autonomous enabled
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Set coast mode with override
    setBrakeMode(!coastSupplier.getAsBoolean());

    // Don't run profile when characterizing, coast mode, or disabled
    if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
      // Run closed loop
      goalHeight = goal.getRads();
      if (goal == Goal.STOW) {
        goalHeight = getStowHeight();
      }
      setpointState =
          profile.calculate(
              Constants.loopPeriodSecs,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goalHeight,
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));
      if (goal == Goal.STOW && EqualsUtil.epsilonEquals(goalHeight, minHeight) && atGoal()) {
        io.stop();
      } else {
        io.runSetpoint(
            setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
      }
    }
    Logger.recordOutput("Elevator/SetpointHeight", setpointState.position);
    Logger.recordOutput("Elevator/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Superstructure/Elevator/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalHeight, 1e-3);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void setProfileConstraints(TrapezoidProfile.Constraints constraints) {
    if (EqualsUtil.epsilonEquals(currentConstraints.maxVelocity, constraints.maxVelocity)
        && EqualsUtil.epsilonEquals(currentConstraints.maxAcceleration, constraints.maxVelocity))
      return;
    currentConstraints = constraints;
    profile = new TrapezoidProfile(currentConstraints);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }
}
