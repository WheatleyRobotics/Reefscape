// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public static final double sprocketRadius = Units.inchesToMeters(1.756 / 2.0);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 0.685);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 2);
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingVelocityThresh", 5.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", 1);

  static {
    switch (Constants.getRobotType()) {
      case COMPBOT, DEVBOT -> {
        kP.initDefault(360);
        kI.initDefault(0);
        kD.initDefault(40);
        kS.initDefault(1.9);
        kG.initDefault(0.0);
      }
      case SIMBOT -> {
        kP.initDefault(3000);
        kI.initDefault(0);
        kD.initDefault(800);
        kS.initDefault(5);
        kG.initDefault(50);
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @AutoLogOutput(key = "Elevator/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput @Getter private boolean homed = false;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private boolean stowed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.motorConnected);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && homed
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);
    // Check if out of tolerance
    boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.get().position, 0.0, SuperstructureConstants.elevatorMaxTravel),
              goal.get().velocity);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0
          || setpoint.position > SuperstructureConstants.elevatorMaxTravel) {
        setpoint =
            new State(
                MathUtil.clamp(setpoint.position, 0.0, SuperstructureConstants.elevatorMaxTravel),
                0.0);
      }
      io.runPosition(
          setpoint.position / sprocketRadius + homedPosition,
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * SuperstructureConstants.elevatorAngle.getSin());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Stop running elevator down when in stow
      if (stowed && atGoal) {
        io.stop();
      }

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }
    if (isEStopped) {
      io.stop();
    }

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec", inputs.velocityRadPerSec * sprocketRadius);
  }

  public void setGoal(DoubleSupplier goal) {
    setGoal(() -> new State(goal.getAsDouble(), 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    atGoal = false;
    this.goal = goal;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> inputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              homedPosition = inputs.positionRad;
              homed = true;
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  /** Get position of elevator in meters with 0 at home */
  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.positionRad - homedPosition) * sprocketRadius;
  }

  public double getGoalMeters() {
    return goal.get().position;
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void runOpenLoop(double amps) {
    io.runOpenLoop(amps);
  }
}
