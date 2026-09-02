package frc.lib;

/**
 * Closed-loop gains for a single mechanism, in the same layout as a Phoenix 6 slot config.
 *
 * <p>These are output-unit agnostic: the meaning of each term depends on the control request the IO
 * layer uses. For the turret's torque-current FOC requests the output unit is <b>amps</b>, so:
 *
 * <ul>
 *   <li>{@code kP} — amps per rotation (position loops) or amps per rotation/sec (velocity loops)
 *   <li>{@code kI} — amps per rotation-second
 *   <li>{@code kD} — amps per rotation/sec (position loops)
 *   <li>{@code kS} — amps needed to overcome static friction
 *   <li>{@code kV} — amps per rotation/sec. Normally <b>0</b> under torque control: a torque output
 *       of zero already holds a constant velocity when there are no external forces.
 *   <li>{@code kA} — amps per rotation/sec²; this is just the mechanism's moment of inertia
 *       expressed in motor current, and is the natural feedforward term for torque control.
 *   <li>{@code kG} — amps needed to hold the mechanism against gravity
 * </ul>
 */
public record Gains(
    double kP, double kI, double kD, double kS, double kV, double kA, double kG) {

  public static final Gains kZero = new Gains(0, 0, 0, 0, 0, 0, 0);

  public Gains withKP(double kP) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKI(double kI) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKD(double kD) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKS(double kS) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKV(double kV) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKA(double kA) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }

  public Gains withKG(double kG) {
    return new Gains(kP, kI, kD, kS, kV, kA, kG);
  }
}
