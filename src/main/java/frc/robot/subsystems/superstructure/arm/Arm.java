package frc.robot.subsystems.superstructure.arm;

import static frc.robot.subsystems.superstructure.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
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

public class Arm {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber smoothVelocity =
      new LoggedTunableNumber("Arm/SmoothVelocity", profileConstraints.maxVelocity * 0.75);
  private static final LoggedTunableNumber smoothAcceleration =
      new LoggedTunableNumber("Arm/SmoothAcceleration", profileConstraints.maxAcceleration * 0.5);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxAngle.getDegrees());

  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> smoothProfileConstraints =
      () -> new TrapezoidProfile.Constraints(smoothVelocity.get(), smoothAcceleration.get());

  @RequiredArgsConstructor
  public enum Goal {
    STOW(() -> 0),
    L1(new LoggedTunableNumber("Arm/L1", 40.0)),
    L2(new LoggedTunableNumber("Arm/L2", 45.0)),
    L3(new LoggedTunableNumber("Arm/L3", 110.0)),
    L4(new LoggedTunableNumber("Arm/L4", 55.0));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  @AutoLogOutput @Setter private double currentCompensation = 0.0;
  private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.kWarning);

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;

  private boolean brakeModeEnabled = true;
  private boolean wasNotAuto = false;

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
  }

  public void setOverrides(BooleanSupplier disableOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
  }

  private double getStowAngle() {
    return Goal.STOW.getRads();
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.armMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    if (disableSupplier.getAsBoolean()
        || (Constants.getMode() == Constants.Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto)) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
    }

    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Don't run profile when characterizing, coast mode, or disabled
    if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
      // Run closed loop
      goalAngle = goal.getRads();
      if (goal == Goal.STOW) {
        goalAngle = getStowAngle();
      }
      setpointState =
          profile.calculate(
              Constants.loopPeriodSecs,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goalAngle,
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));
      if (goal == Goal.STOW
          && EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians())
          && atGoal()) {
        io.stop();
      } else {
        io.runSetpoint(
            setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
      }

      goalVisualizer.update(goalAngle);
      Logger.recordOutput("Arm/GoalAngle", goalAngle);
    }

    // Logs
    measuredVisualizer.update(inputs.positionRads);
    setpointVisualizer.update(setpointState.position);
    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
    Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Superstructure/Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
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
    return inputs.velocityRadsPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }
}
