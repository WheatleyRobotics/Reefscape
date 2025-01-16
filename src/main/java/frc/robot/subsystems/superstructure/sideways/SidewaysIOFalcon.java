// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.sideways;

import frc.robot.subsystems.superstructure.elevator.ElevatorIOFalcon;

public class SidewaysIOFalcon extends ElevatorIOFalcon implements SidewaysIO {
  private static final int id = 5;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = true;
  private static final double reduction = 60.0 / 1.0;

  public SidewaysIOFalcon() {
    super(id, bus, currentLimitAmps, invert, reduction);
  }
}
