// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class Elevator<G extends Elevator.SlamElevatorGoal> {

  public interface SlamElevatorGoal {
    DoubleSupplier getSlammingCurrent();

    boolean isStopAtGoal();

    default boolean isNonSensing() {
      return false;
    }

    SlamElevatorState getState();
  }

  public enum SlamElevatorState {
    IDLING,
    RETRACTING,
    EXTENDING
  }

  private final ElevatorIO io;
  protected final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final String name;
  private final double staticTimeSecs;
  private final double minVelocityThresh;

  protected abstract G getGoal();

  private G lastGoal = null;

  private boolean atGoal = false;
  private final Timer staticTimer = new Timer();

  private boolean brakeModeEnabled = false;
  private BooleanSupplier coastModeSupplier = () -> false;

  private final Alert disconnected;

  /**
   * Creates a new GenericSlamElevator
   *
   * @param name Name of elevator.
   * @param io IO implementation of elevator.
   * @param staticTimeSecs Time that it takes for elevator to stop running after hitting the end of
   *     the elevator.
   * @param minVelocityThresh Minimum velocity threshold for elevator to start stopping at in
   *     rads/sec of the last sprocket.
   */
  public Elevator(String name, ElevatorIO io, double staticTimeSecs, double minVelocityThresh) {
    this.name = name;
    this.io = io;
    this.staticTimeSecs = staticTimeSecs;
    this.minVelocityThresh = minVelocityThresh;
    setBrakeMode(true);

    disconnected = new Alert(name + " disconnected!", Alert.AlertType.kWarning);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    coastModeSupplier = coastOverride;
  }

  private void setBrakeMode(boolean enable) {
    if (brakeModeEnabled == enable) return;
    brakeModeEnabled = enable;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Ensure brake mode is enabled
    if (DriverStation.isEnabled()) {
      setBrakeMode(true);
    }

    // Reset if changing goals
    if (lastGoal != null && getGoal() != lastGoal) {
      atGoal = false;
      staticTimer.stop();
      staticTimer.reset();
    }
    // Set last goal
    lastGoal = getGoal();

    // Set alert
    disconnected.set(!inputs.motorConnected);

    // Check if at goal.
    if (!atGoal) {
      // Start static timer if within min velocity threshold.
      if (getGoal().isNonSensing() || Math.abs(inputs.velocityRadsPerSec) <= minVelocityThresh) {
        staticTimer.start();
      } else {
        staticTimer.stop();
        staticTimer.reset();
      }
      // If we are finished with timer, finish goal.
      // Also assume we are at the goal if auto was started
      atGoal = staticTimer.hasElapsed(staticTimeSecs) || DriverStation.isAutonomousEnabled();
    } else {
      staticTimer.stop();
      staticTimer.reset();
    }

    // Run to goal.
    if (!atGoal) {
      io.runVoltage(getGoal().getSlammingCurrent().getAsDouble());
    } else {
      if (getGoal().isStopAtGoal()) {
        io.stop();
      } else {
        io.runVoltage(getGoal().getSlammingCurrent().getAsDouble());
      }
    }

    if (DriverStation.isDisabled()) {
      // Reset
      io.stop();
      lastGoal = null;
      staticTimer.stop();
      staticTimer.reset();
      if (Math.abs(inputs.velocityRadsPerSec) > minVelocityThresh) {
        // If we don't move when disabled, assume we are still at goal
        atGoal = false;
      }
    }

    // Update coast mode
    setBrakeMode(!coastModeSupplier.getAsBoolean());

    Logger.recordOutput("Superstructure/" + name + "/Goal", getGoal().toString());
  }

  @AutoLogOutput(key = "Superstructure/{name}/AtGoal")
  public boolean atGoal() {
    return atGoal;
  }

  @AutoLogOutput(key = "Superstructure/{name}/Extended")
  public boolean extended() {
    return getGoal().getState() == SlamElevatorState.EXTENDING && atGoal;
  }

  @AutoLogOutput(key = "Superstructure/{name}/Retracted")
  public boolean retracted() {
    return getGoal().getState() == SlamElevatorState.RETRACTING && atGoal;
  }
}
