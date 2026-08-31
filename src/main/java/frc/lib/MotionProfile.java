package frc.lib;

/**
 * Motion Magic constraints for a mechanism, in mechanism rotations.
 *
 * @param cruiseVelocity peak velocity of the profile, rotations/sec
 * @param acceleration peak acceleration of the profile, rotations/sec²
 * @param jerk peak jerk of the profile, rotations/sec³. Zero disables jerk limiting.
 */
public record MotionProfile(double cruiseVelocity, double acceleration, double jerk) {

  public MotionProfile(double cruiseVelocity, double acceleration) {
    this(cruiseVelocity, acceleration, 0);
  }
}
