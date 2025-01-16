// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.sideways;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;

/** Add your docs here. */
public class SidewaysIOSim extends ElevatorIOSim implements SidewaysIO {
  private static final double maxLengthMeters = Units.inchesToMeters(15.25);
  private static final double reduction = 60.0 / 1.0;
  private static final double drumRadiusMeters = Units.inchesToMeters(1.275);

  public SidewaysIOSim() {
    super(maxLengthMeters, reduction, drumRadiusMeters);
  }
}
