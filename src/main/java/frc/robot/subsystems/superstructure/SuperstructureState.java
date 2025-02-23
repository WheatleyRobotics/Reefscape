// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.SuperstructureStateData.Height;
import frc.robot.subsystems.superstructure.dispenser.Dispenser;
import frc.robot.subsystems.superstructure.slam.Slam;
import frc.robot.subsystems.superstructure.slam.Slam.Goal;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  START(SuperstructureStateData.builder().build()),
  STOW(SuperstructureStateData.builder().pose(SuperstructurePose.Preset.STOW.getPose()).build()),
  INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.START.getPose())
          .tunnelVolts(Dispenser.tunnelIntakeVolts)
          .build()),
  L1_CORAL(SuperstructureStateData.builder().pose(SuperstructurePose.Preset.L1.getPose()).build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L2.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L3.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L4_CORAL(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.L4.getPose())
          .height(Height.SECOND_STAGE)
          .build()),
  L1_CORAL_EJECT(
      L1_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L2_CORAL_EJECT(
      L2_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L3_CORAL_EJECT(
      L3_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L4_CORAL_EJECT(
      L4_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  ALGAE_FLOOR_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_FLOOR_INTAKE.getPose())
          .slamGoal(Slam.Goal.SLAM_DOWN)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .intakeVolts(Slam.floorIntakeVolts)
          .height(Height.BOTTOM)
          .build()),
  ALGAE_L2_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_L2_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L3_INTAKE(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_L3_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  THROWN(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.THROW.getPose())
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  ALGAE_STOW(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.ALGAE_STOW.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .slamGoal(Slam.Goal.SLAM_DOWN)
          .height(Height.INTAKE)
          .build()),
  TOSS(
      SuperstructureStateData.builder()
          .pose(
              new SuperstructurePose(
                  SuperstructurePose.Preset.ALGAE_STOW.getPose().elevatorHeight(),
                  () -> Rotation2d.fromDegrees(30.0)))
          .slamGoal(Goal.SLAM_UP)
          .height(Height.INTAKE)
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .build()),
  PROCESSING(
      SuperstructureStateData.builder()
          .pose(SuperstructurePose.Preset.PROCESSING.getPose())
          .slamGoal(Goal.SLAM_UP)
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .intakeVolts(Slam.occupiedVolts)
          .build());
  private final SuperstructureStateData value;

  public SuperstructureState getEject() {
    SuperstructureState returnState;
    try {
      if (this.name().equals("STOW")) {
        returnState = SuperstructureState.L1_CORAL_EJECT;
      } else {
        returnState = SuperstructureState.valueOf(this.name() + "_EJECT");
      }
    } catch (IllegalArgumentException e) {
      returnState = this;
    }
    System.out.println("Ejecting " + this.name() + " to " + returnState.name());
    return returnState;
  }
}
