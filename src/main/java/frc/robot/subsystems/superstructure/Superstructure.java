// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  @NoArgsConstructor(force = true)
  @RequiredArgsConstructor
  @Getter
  public enum Goal {
    STOW,
    L1,
    L2,
    L3,
    L4;

    private final boolean climbingGoal;
  }

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;
  private Goal lastGoal = Goal.STOW;

  private final Arm arm;
  private final Elevator elevator;

  private Timer goalTimer = new Timer();

  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;

    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
      arm.stop();
    }

    // Reset timer
    if (currentGoal != lastGoal) {
      goalTimer.reset();
    }
    lastGoal = currentGoal;

    switch (currentGoal) {
      case STOW -> {
        arm.setGoal(Arm.Goal.L4);
        elevator.setGoal(Elevator.Goal.L4);
      }
      case L1 -> {
        arm.setGoal(Arm.Goal.L1);
        elevator.setGoal(Elevator.Goal.L1);
      }
      case L2 -> {
        arm.setGoal(Arm.Goal.L2);
        elevator.setGoal(Elevator.Goal.L2);
      }
      case L3 -> {
        arm.setGoal(Arm.Goal.L3);
        elevator.setGoal(Elevator.Goal.L3);
      }
      case L4 -> {
        arm.setGoal(Arm.Goal.L4);
        elevator.setGoal(Elevator.Goal.L4);
      }
    }

    arm.periodic();
    elevator.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  /** Set goal of superstructure */
  private void setGoal(Goal goal) {
    if (desiredGoal == goal) return;
    desiredGoal = goal;
  }

  /** Command to set goal of superstructure */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
        .withName("Superstructure " + goal);
  }

  /** Command to set goal of superstructure with additional profile constraints on arm */
  public Command setGoalWithConstraintsCommand(
      Goal goal, TrapezoidProfile.Constraints armProfileConstraints) {
    return setGoalCommand(goal)
        .beforeStarting(() -> arm.setProfileConstraints(armProfileConstraints))
        .finallyDo(() -> arm.setProfileConstraints(Arm.maxProfileConstraints.get()));
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() && elevator.atGoal();
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && arm.atGoal();
  }

  public void runArmCharacterization(double input) {
    arm.runCharacterization(input);
  }

  public double getArmCharacterizationVelocity() {
    return arm.getCharacterizationVelocity();
  }

  public void endArmCharacterization() {
    arm.endCharacterization();
  }
}
