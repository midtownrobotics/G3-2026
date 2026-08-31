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

  /**
   * Kraken X60 winding resistance with FOC, from its 12 V / 483 A stall point. Near zero speed —
   * where back-EMF is negligible — an applied volt produces roughly {@code 1 / kKrakenX60FocOhms}
   * amps.
   */
  private static final double kKrakenX60FocOhms = 12.0 / 483.0;

  /**
   * Rescales gains that were tuned against a voltage output into the amp units a torque-current FOC
   * request expects, by dividing through by the motor's winding resistance.
   *
   * <p>This is a first-order estimate only. It ignores back-EMF, so it is most accurate for the
   * terms that dominate near zero velocity ({@code kP}, {@code kS}, {@code kG}) and least accurate
   * for {@code kV}. Treat the result as a starting point for retuning on the robot, not as a
   * finished set of gains.
   *
   * <p>Note that {@code kV} usually belongs at zero under torque control regardless of what it was
   * under voltage control — see {@link Gains}.
   */
  public static Gains fromKrakenVoltageGains(Gains voltageGains) {
    return new Gains(
        voltageGains.kP() / kKrakenX60FocOhms,
        voltageGains.kI() / kKrakenX60FocOhms,
        voltageGains.kD() / kKrakenX60FocOhms,
        voltageGains.kS() / kKrakenX60FocOhms,
        voltageGains.kV() / kKrakenX60FocOhms,
        voltageGains.kA() / kKrakenX60FocOhms,
        voltageGains.kG() / kKrakenX60FocOhms);
  }

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
