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
import frc.robot.subsystems.superstructure.sideways.Sideways;
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
    BACKPACK_OUT_UNJAM,
    AIM,
    SUPER_POOP,
    AIM_AT_DEMO_TAG,
    DEMO_SHOT,
    UNJAM_FEEDER,
    STATION_INTAKE,
    AMP,
    SUBWOOFER,
    PODIUM,
    RESET_CLIMB(true),
    PREPARE_PREPARE_TRAP_CLIMB(true),
    PREPARE_CLIMB(true),
    POST_PREPARE_TRAP_CLIMB(true),
    CLIMB(true),
    TRAP(true),
    UNTRAP(true),
    RESET,
    DIAGNOSTIC_ARM;

    private final boolean climbingGoal;
  }

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;
  private Goal lastGoal = Goal.STOW;

  private final Arm arm;
  private final Sideways sideways;

  private Timer goalTimer = new Timer();

  public Superstructure(Arm arm, Sideways sideways) {
    this.arm = arm;
    this.sideways = sideways;
    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
      arm.stop();
    }

    // Retract climber
    if (!sideways.retracted()
        && !desiredGoal.isClimbingGoal()
        && !DriverStation.isAutonomousEnabled()) {
      currentGoal = Goal.RESET_CLIMB;
    } else {
      currentGoal = desiredGoal;
    }

    // Reset timer
    if (currentGoal != lastGoal) {
      goalTimer.reset();
    }
    lastGoal = currentGoal;

    switch (currentGoal) {
      case STOW -> {
        arm.setGoal(Arm.Goal.STOW);
        sideways.setGoal(Sideways.Goal.IDLE);
      }
      case STATION_INTAKE -> {
        arm.setGoal(Arm.Goal.STATION_INTAKE);
        sideways.setGoal(Sideways.Goal.IDLE);
      }
      case RESET_CLIMB -> {
        arm.setGoal(Arm.Goal.RESET_CLIMB);
        if (arm.atGoal()) {
          // Retract and then stop
          sideways.setGoal(Sideways.Goal.IDLE);
        } else {
          // Arm in unsafe state to retract, apply no current
          sideways.setGoal(Sideways.Goal.STOP);
        }
      }
      case PREPARE_CLIMB -> {
        arm.setGoal(Arm.Goal.PREPARE_CLIMB);
        sideways.setGoal(Sideways.Goal.EXTEND);
      }
      case RESET -> {
        desiredGoal = Goal.STOW;
        setDefaultCommand(setGoalCommand(Goal.STOW));
      }
      case DIAGNOSTIC_ARM -> {
        arm.setGoal(Arm.Goal.CUSTOM);
        sideways.setGoal(Sideways.Goal.IDLE);
      }
    }

    arm.periodic();
    sideways.periodic();

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

  /** Command to aim the superstructure with a compensation value in degrees */
  public Command aimWithCompensation(double compensation) {
    return setGoalCommand(Goal.AIM)
        .beforeStarting(() -> arm.setCurrentCompensation(compensation))
        .finallyDo(() -> arm.setCurrentCompensation(0.0));
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal && arm.atGoal() && sideways.atGoal();
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
