// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.slam;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.superstructure.dispenser.TunnelIO;
import frc.robot.subsystems.superstructure.dispenser.TunnelIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Slam {
  private static final LoggedTunableNumber slammedVelocityThresh =
      new LoggedTunableNumber("Slam/SlammedVelocityThresh", 0.1);
  private static final LoggedTunableNumber slammedTimeSecs =
      new LoggedTunableNumber("Slam/SlammedTimeSecs", 0.3);
  private static final LoggedTunableNumber slamDownCurrent =
      new LoggedTunableNumber("Slam/SlamDownCurrent", -40.0);
  private static final LoggedTunableNumber slamUpCurrent =
      new LoggedTunableNumber("Slam/SlamUpCurrent", 40.0);
  public static final LoggedTunableNumber occupiedVolts =
      new LoggedTunableNumber("Slam/OccupiedVolts", 5.0);
  public static final LoggedTunableNumber processorVolts =
      new LoggedTunableNumber("Slam/ProcessorVolts", -5.0);
  public static final LoggedTunableNumber floorIntakeVolts =
      new LoggedTunableNumber("Slam/FloorIntakeVolts", 8.0);

  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(90.0);
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(40.0);

  @Getter
  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, false),
    SLAM_DOWN(slamDownCurrent, false),
    SLAM_UP(slamUpCurrent, true);

    private final DoubleSupplier slamCurrent;
    private final boolean retracted;
  }

  private final SlamIO slamIO;
  private final SlamIOInputsAutoLogged slamInputs = new SlamIOInputsAutoLogged();
  private final TunnelIO rollerIO;
  private final TunnelIOInputsAutoLogged rollerInputs = new TunnelIOInputsAutoLogged();

  @Setter @Getter @AutoLogOutput private Goal goal = Goal.SLAM_UP;
  @Setter private double intakeVolts = 0.0;
  private Goal lastGoal = null;
  @Getter @AutoLogOutput private boolean retracting = false;
  @Getter @AutoLogOutput private boolean slammed = false;

  private Debouncer slamDebouncer = new Debouncer(0.0);

  @Setter private BooleanSupplier coastOverride = () -> false;

  public Slam(SlamIO slamIO, TunnelIO rollerIO) {
    this.slamIO = slamIO;
    this.rollerIO = rollerIO;
  }

  public void periodic() {
    slamIO.updateInputs(slamInputs);
    Logger.processInputs("Slam/SlamInputs", slamInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Slam/RollerInputs", rollerInputs);

    // Set brake mode
    slamIO.setBrakeMode(!coastOverride.getAsBoolean());

    if (lastGoal == null || lastGoal != goal) {
      slammed = false;
      slamDebouncer = new Debouncer(slammedTimeSecs.getAsDouble());
      slamDebouncer.calculate(false);
      lastGoal = goal;
    } else {
      if (slamDebouncer.calculate(
          Math.abs(slamInputs.velocityRadPerSec) <= slammedVelocityThresh.get())) {
        slammed = true;
        retracting = goal.isRetracted();
      }

      if (!slammed) {
        slamIO.runTorqueCurrent(goal.getSlamCurrent().getAsDouble());
      } else {
        slamIO.stop();
      }
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      slammed = false;
      slamDebouncer.calculate(false);
    }

    rollerIO.runVolts(intakeVolts);

    Logger.recordOutput("Slam/BrakeModeEnabled", !coastOverride.getAsBoolean());
  }
}
