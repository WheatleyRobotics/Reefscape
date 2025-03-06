package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final WinchIO winchIO;
  private final Servo servo;
  private final WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();

  public Climb(WinchIO winchIO) {
    this.winchIO = winchIO;
    this.servo = new Servo(1);
    servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void periodic() {
    winchIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void runVolts(double volts) {
    winchIO.runVolts(volts);
  }

  /**
   * @param position
   *     <p>Sets the position of the servo. 1 for extend, 0 for retract
   */
  public void setServoPosition(double position) {
    servo.setPosition(position);
  }

  public void stop() {
    winchIO.stop();
  }
}
