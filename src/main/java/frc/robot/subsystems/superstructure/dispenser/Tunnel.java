// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.dispenser;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Tunnel extends SubsystemBase {
  private final TunnelIO io;
  protected final TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();

  public Tunnel(TunnelIO io) {
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
  
  @AutoLogOutput
  public Command runRollerVelocity(double velocityRadsPerSec) {
    return startEnd(() -> io.runVelocity(velocityRadsPerSec), () -> io.stop());
  }
  
  public void setPIDGains(double kP, double kI, double kD, double kF) {
    io.setPID(kP, kI, kD, kF);
  }
}
