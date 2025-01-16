// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.sideways;

import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
public class Sideways extends Elevator<Sideways.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements Elevator.SlamElevatorGoal {
    STOP(new LoggedTunableNumber("Climber/StopCurrent", 0.0), false, SlamElevatorState.IDLING),
    IDLE(new LoggedTunableNumber("Climber/IdleCurrent", -12.0), true, SlamElevatorState.RETRACTING),
    RETRACT(
        new LoggedTunableNumber("Climber/RetractingCurrent", -40.0),
        false,
        SlamElevatorState.RETRACTING),
    EXTEND(
        new LoggedTunableNumber("Climber/ExtendingCurrent", 12.0),
        true,
        SlamElevatorState.EXTENDING);

    private final DoubleSupplier slammingCurrent;
    private final boolean stopAtGoal;
    private final SlamElevatorState state;
  }

  private Goal goal = Goal.IDLE;

  public Sideways(SidewaysIO io) {
    super("Sidewats Elevator", io, 0.4, 1.5);
  }
}
