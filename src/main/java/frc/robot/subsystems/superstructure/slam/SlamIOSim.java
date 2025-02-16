// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.slam;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureVisualizer;

public class SlamIOSim implements SlamIO {
  private static final double moi = 1.0;
  private static final double cgRadius = 0.2;
  private static final DCMotor gearbox =
      DCMotor.getKrakenX60Foc(1).withReduction(SlamIOTalonFX.reduction);
  private static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  private static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  // Slam sim
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  public SlamIOSim() {
    simState = VecBuilder.fill(Math.PI / 2.0, 0.0);
  }

  @Override
  public void updateInputs(SlamIOInputs inputs) {
    update(Constants.loopPeriodSecs);
    SuperstructureVisualizer.updateSimIntake(simState.get(0));
    inputs.positionRad = simState.get(0);
    inputs.velocityRadPerSec = simState.get(1);
    inputs.appliedVolts = appliedVolts;
    inputs.torqueCurrentAmps = inputTorqueCurrent;
  }

  @Override
  public void runTorqueCurrent(double current) {
    inputTorqueCurrent = current;
    appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    runTorqueCurrent(0.0);
  }

  private void update(double dt) {
    inputTorqueCurrent =
        MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                A.times(x)
                    .plus(B.times(u))
                    .plus(
                        -SuperstructureConstants.G
                            * cgRadius
                            * Rotation2d.fromRadians(simState.get(0)).getCos()
                            / moi),
            simState,
            VecBuilder.fill(inputTorqueCurrent * 15), // Magic constant of doom
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= Slam.minAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, Slam.minAngle.getRadians());
    }
    if (simState.get(0) >= Slam.maxAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, Slam.maxAngle.getRadians());
    }
  }
}
