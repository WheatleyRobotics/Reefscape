// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.slam.Slam;
import java.util.function.DoubleSupplier;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
  @Builder.Default private final SuperstructurePose pose = new SuperstructurePose();
  @Builder.Default private final DoubleSupplier tunnelVolts = () -> 0.0;
  @Builder.Default private final DoubleSupplier gripperCurrent = () -> 0.0;
  @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;
  @Builder.Default private final Slam.Goal slamGoal = Slam.Goal.SLAM_UP;
  @Builder.Default private final Height height = Height.BOTTOM;
  @Builder.Default private final boolean reversed = false;

  /** What height is the carriage above? */
  @RequiredArgsConstructor
  public enum Height {
    BOTTOM(1),
    INTAKE(2),
    FIRST_STAGE(3),
    SECOND_STAGE(4);

    private final int order;

    public boolean lowerThan(Height other) {
      return order <= other.order;
    }

    public boolean upperThan(Height other) {
      return order > other.order;
    }
  }
}
