package frc.lib;

import java.util.function.Consumer;

/**
 * A dashboard-tunable set of {@link Gains}.
 *
 * <p>Publishes one {@link LoggedTunableNumber} per term under {@code /Tuning/<key>/…} and hands the
 * whole set to a consumer whenever any term changes. The consumer also fires once on the first
 * {@link #poll} so the configured defaults are pushed to the device at startup — which means the
 * dashboard is always the single source of truth for the gains, with no way for the device config
 * and the displayed numbers to disagree.
 *
 * <p>When {@link frc.robot.constants.Constants#kTuningMode} is false the terms are not published and
 * the defaults are applied once, so competition builds pay no NetworkTables cost.
 *
 * <pre>
 * // in the subsystem constructor
 * m_gains = new TunableGains("Turret", TurretIOTalonFX.kDefaultGains);
 *
 * // in periodic()
 * m_gains.poll(hashCode(), m_io::setGains);
 * </pre>
 */
public class TunableGains {
  private final LoggedTunableNumber m_kP;
  private final LoggedTunableNumber m_kI;
  private final LoggedTunableNumber m_kD;
  private final LoggedTunableNumber m_kS;
  private final LoggedTunableNumber m_kV;
  private final LoggedTunableNumber m_kA;
  private final LoggedTunableNumber m_kG;

  /**
   * @param key dashboard key prefix, typically the subsystem name
   * @param defaults values used before the dashboard is touched, and the only values used when not
   *     in tuning mode
   */
  public TunableGains(String key, Gains defaults) {
    m_kP = new LoggedTunableNumber(key + "/Gains/kP", defaults.kP());
    m_kI = new LoggedTunableNumber(key + "/Gains/kI", defaults.kI());
    m_kD = new LoggedTunableNumber(key + "/Gains/kD", defaults.kD());
    m_kS = new LoggedTunableNumber(key + "/Gains/kS", defaults.kS());
    m_kV = new LoggedTunableNumber(key + "/Gains/kV", defaults.kV());
    m_kA = new LoggedTunableNumber(key + "/Gains/kA", defaults.kA());
    m_kG = new LoggedTunableNumber(key + "/Gains/kG", defaults.kG());
  }

  /** The gains currently shown on the dashboard. */
  public Gains get() {
    return new Gains(
        m_kP.get(), m_kI.get(), m_kD.get(), m_kS.get(), m_kV.get(), m_kA.get(), m_kG.get());
  }

  /**
   * Applies the gains if any term has changed since the last poll. Call this once per loop from
   * {@code periodic()}.
   *
   * @param id unique caller id; pass {@code hashCode()}
   * @param apply receives the new gains, typically {@code m_io::setGains}
   */
  public void poll(int id, Consumer<Gains> apply) {
    LoggedTunableNumber.ifChanged(
        id, () -> apply.accept(get()), m_kP, m_kI, m_kD, m_kS, m_kV, m_kA, m_kG);
  }
}
