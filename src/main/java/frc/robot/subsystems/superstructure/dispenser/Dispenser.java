// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.dispenser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.roller.RollerSystemIO;
import frc.robot.subsystems.superstructure.roller.RollerSystemIOInputsAutoLogged;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Dispenser {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(18);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(180 + 34);
  private static final double maxAngleRad = calculateFinalAngle(maxAngle).getRadians();
  private static final double minAngleRad = calculateFinalAngle(minAngle).getRadians();

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Dispenser/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Dispenser/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Dispenser/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Dispenser/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Dispenser/MaxVelocityDegreesPerSec", 15);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Dispenser/MaxAccelerationDegreesPerSec2", 60);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Dispenser/StaticCharacterizationVelocityThresh", 0.1);
  private static final LoggedTunableNumber algaeIntakeCurrentThresh =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeCurrentThreshold", 40.0);
  public static final LoggedTunableNumber gripperIntakeCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeIntakeCurrent", 30.0);
  public static final LoggedTunableNumber gripperDispenseCurrent =
      new LoggedTunableNumber("Dispenser/AlgaeDispenseCurrent", -30.0);
  public static final LoggedTunableNumber tunnelDispenseVolts =
      new LoggedTunableNumber("Dispenser/TunnelDispenseVolts", -6.0);
  public static final LoggedTunableNumber tunnelIntakeVolts =
      new LoggedTunableNumber("Dispenser/TunnelIntakeVolts", 6.0);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Dispenser/Tolerance", 0.2);

  static {
    switch (Constants.getRobotType()) {
      case SIMBOT -> {
        kP.initDefault(7000);
        kD.initDefault(2000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
      default -> {
        kP.initDefault(0);
        kD.initDefault(0);
        kS.initDefault(27.163);
        kG.initDefault(0);
      }
    }
  }

  // Hardware
  private final DispenserIO pivotIO;
  private final DispenserIOInputsAutoLogged pivotInputs = new DispenserIOInputsAutoLogged();
  private final RollerSystemIO effectorIO;
  private final RollerSystemIOInputsAutoLogged effectorInputs =
      new RollerSystemIOInputsAutoLogged();

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput(key = "Dispenser/PivotBrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  // Control
  @Getter
  @AutoLogOutput(key = "Dispenser/MeasuredAngle")
  private Rotation2d finalAngle = new Rotation2d();

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @Getter
  @AutoLogOutput(key = "Dispenser/Profile/AtGoal")
  private boolean atGoal = false;

  @Getter @AutoLogOutput private boolean hasAlgae = false;

  @Getter @AutoLogOutput private boolean hasCoral = false;

  @Setter private double tunnelVolts = 0.0;
  @Setter private double gripperCurrent = 0.0;

  private Debouncer algaeDebouncer = new Debouncer(0.1);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  // Disconnected alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Dispenser pivot motor disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Dispenser encoder disconnected!", Alert.AlertType.kWarning);
  private final Alert talonDisconnectedAlert =
      new Alert("Dispenser tunnel disconnected!", Alert.AlertType.kWarning);
  private final Alert canRangeDisconnectedAlart =
      new Alert("Dispenser CANRange disconnected!", Alert.AlertType.kWarning);

  public Dispenser(DispenserIO pivotIO, RollerSystemIO effectorIO) {
    this.pivotIO = pivotIO;
    this.effectorIO = effectorIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));

    if (Constants.getRobotType() == Constants.RobotType.SIMBOT) {
      new Trigger(() -> DriverStation.getStickButtonPressed(2, 1))
          .onTrue(Commands.runOnce(() -> hasAlgae = !hasAlgae));
      new Trigger(() -> DriverStation.getStickButtonPressed(2, 2))
          .onTrue(Commands.runOnce(() -> hasCoral = !hasCoral));
    }
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Dispenser/Pivot", pivotInputs);
    effectorIO.updateInputs(effectorInputs);
    Logger.processInputs("Dispenser/Effector", effectorInputs);

    pivotMotorDisconnectedAlert.set(
        !pivotInputs.motorConnected && Constants.getRobotType() == RobotType.COMPBOT);
    pivotEncoderDisconnectedAlert.set(
        !pivotInputs.encoderConnected && Constants.getRobotType() == RobotType.COMPBOT);
    talonDisconnectedAlert.set(!effectorInputs.talonConnected);
    canRangeDisconnectedAlart.set(!effectorInputs.CANRangeConnected);

    if (Constants.getRobotType() != RobotType.SIMBOT) {
      hasCoral = effectorInputs.hasCoral;
    }

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      pivotIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Calculate combined angle
    finalAngle = calculateFinalAngle(pivotInputs.internalPosition);

    // Run profile
    boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Dispenser/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(finalAngle.getRadians() - setpoint.position) > tolerance.get();
    shouldEStop = false;
    /*
       toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile)
           || finalAngle.getRadians() < minAngleRad
           || finalAngle.getRadians() > maxAngleRad;

    */
    shouldRunProfile = false;
    if (shouldRunProfile) {
      // Clamp goal
      var goalState = new State(MathUtil.clamp(goal.getAsDouble(), minAngleRad, maxAngleRad), 0.0);
      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
      pivotIO.runPosition(
          Rotation2d.fromRadians(
              setpoint.position + SuperstructureConstants.elevatorAngle.getRadians()),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * finalAngle.getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Handle effector
      /*
           if (!effectorInputs.hasCoral && !hasAlgae) {
             tunnelVolts = tunnelIntakeVolts.getAsDouble();
             gripperCurrent = 0;
           } else if (hasAlgae && !effectorInputs.hasCoral) {
             gripperCurrent = gripperIntakeCurrent.getAsDouble();
             tunnelVolts = 0;
           } else {
             tunnelVolts = 0;
             gripperCurrent = 0;
           }

      */

      // Log state
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", setpoint.position);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", goalState.position);

      Logger.recordOutput("Dispenser/Effector/tunnelVolts", tunnelVolts);
      Logger.recordOutput("Dispenser/Effector/tunnelVolts", gripperCurrent);
    } else {
      // Reset setpoint
      setpoint = new State(finalAngle.getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Dispenser/Profile/SetpointPositionRad", 0.0);
      Logger.recordOutput("Dispenser/Profile/SetpointVelocityRadPerSec", 0.0);
      Logger.recordOutput("Dispenser/Profile/GoalPositionRad", 0.0);

      Logger.recordOutput("Dispenser/Effector/tunnelVolts", 0.0);
      Logger.recordOutput("Dispenser/Effector/tunnelVolts", 0.0);
    }

    // Run tunnel and gripper
    if (!isEStopped) {

    } else {
      pivotIO.stop();
      effectorIO.stop();
    }

    // Check algae
    if (Constants.getRobotType() != Constants.RobotType.SIMBOT) {
      hasAlgae =
          algaeDebouncer.calculate(
              Math.abs(effectorInputs.talonSupplyCurrentAmps) >= algaeIntakeCurrentThresh.get());
    }

    // Log state
    Logger.recordOutput("Dispenser/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Dispenser/DisabledOverride", disabledOverride.getAsBoolean());
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    setGoal(() -> goal.get().getRadians());
  }

  public void setGoal(DoubleSupplier goal) {
    atGoal = false;
    this.goal = goal;
  }

  public double getGoal() {
    return goal.getAsDouble();
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    pivotIO.setBrakeMode(enabled);
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
              pivotIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Dispenser/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> pivotInputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Dispenser/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public static Rotation2d calculateFinalAngle(Rotation2d pivotAngle) {
    return new Rotation2d(
        pivotAngle.getRadians()); // - SuperstructureConstants.elevatorAngle.getRadians());
  }

  public void runVoltsPivot(double volts) {
    pivotIO.runVolts(volts);
  }

  public void runVoltsTunnel(double volts) {
    effectorIO.runVolts(volts);
  }

  public void runCurrentGripper(double amp) {
    effectorIO.runTorqueCurrent(amp);
  }
}
