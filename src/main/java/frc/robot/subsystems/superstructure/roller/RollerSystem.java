// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class RollerSystem extends SubsystemBase {
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();

  public RollerSystem(RollerSystemIO io) {
    this.io = io;
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  @AutoLogOutput
  public Command runRoller(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
  }
}
