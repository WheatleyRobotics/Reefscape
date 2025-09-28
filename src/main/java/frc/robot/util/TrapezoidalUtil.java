package frc.robot.util;

public class TrapezoidalUtil {
  /**
   * Calculates the time to reach setpoint using a trapezoidal velocity profile
   *
   * @param distance Total distance to travel
   * @param initialVel Initial velocity (v0)
   * @param finalVel Final velocity (vf)
   * @param maxVel Maximum velocity (v_max)
   * @param acceleration Acceleration rate (must be positive)
   * @return Total time to reach setpoint
   * @throws IllegalArgumentException if parameters are invalid
   */
  public static double calculateTimeToSetpoint(
      double distance, double initialVel, double finalVel, double maxVel, double acceleration) {

    // Validate inputs
    if (acceleration <= 0) {
      throw new IllegalArgumentException("Acceleration and acceleration must be positive");
    }
    if (distance < 0) {
      throw new IllegalArgumentException("Distance cannot be negative");
    }
    if (distance == 0) {
      return 0.0;
    }

    // Calculate time to accelerate from initial to max velocity
    double t1 = Math.max(0, (maxVel - initialVel) / acceleration);

    // Calculate time to decelerate from max to final velocity
    double t3 = Math.max(0, (maxVel - finalVel) / acceleration);

    // Calculate distances during acceleration and acceleration phases
    double d1 = initialVel * t1 + 0.5 * acceleration * t1 * t1;
    double d3 = finalVel * t3 + 0.5 * acceleration * t3 * t3;

    // Calculate remaining distance for constant velocity phase
    double remainingDistance = distance - d1 - d3;

    // If remaining distance is negative or we can't reach max velocity,
    // we have a triangular profile
    if (remainingDistance <= 0 || maxVel < Math.max(initialVel, finalVel)) {
      // Triangular profile calculation
      double A = 0.5 * acceleration + 0.5 * acceleration * acceleration / acceleration;
      double B = initialVel + acceleration * (initialVel - finalVel) / acceleration;
      double C = 0.5 * (initialVel - finalVel) * (initialVel - finalVel) / acceleration - distance;

      double discriminant = B * B - 4 * A * C;

      if (discriminant < 0) {
        throw new IllegalArgumentException("No valid solution exists for given parameters");
      }

      double t1_tri = (-B + Math.sqrt(discriminant)) / (2 * A);

      if (t1_tri < 0) {
        t1_tri = (-B - Math.sqrt(discriminant)) / (2 * A);
      }

      if (t1_tri < 0) {
        throw new IllegalArgumentException("No positive solution exists");
      }

      double peakVel = initialVel + acceleration * t1_tri;
      double t3_tri = (peakVel - finalVel) / acceleration;

      return t1_tri + t3_tri;
    }

    // Calculate constant velocity time
    double t2 = remainingDistance / maxVel;

    // Total time is sum of all three phases
    return t1 + t2 + t3;
  }
}
