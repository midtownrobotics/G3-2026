package frc.lib;

import java.util.function.Consumer;

/**
 * A dashboard-tunable {@link MotionProfile}, published under {@code /Tuning/<key>/MotionMagic/…}.
 *
 * <p>Behaves like {@link TunableGains}: the consumer fires once on the first {@link #poll} and then
 * only when a constraint changes.
 */
public class TunableMotionProfile {
  private final LoggedTunableNumber m_cruiseVelocity;
  private final LoggedTunableNumber m_acceleration;
  private final LoggedTunableNumber m_jerk;

  public TunableMotionProfile(String key, MotionProfile defaults) {
    m_cruiseVelocity =
        new LoggedTunableNumber(key + "/MotionMagic/CruiseVelocityRotPerSec", defaults.cruiseVelocity());
    m_acceleration =
        new LoggedTunableNumber(key + "/MotionMagic/AccelerationRotPerSecSq", defaults.acceleration());
    m_jerk = new LoggedTunableNumber(key + "/MotionMagic/JerkRotPerSecCubed", defaults.jerk());
  }

  /** The profile currently shown on the dashboard. */
  public MotionProfile get() {
    return new MotionProfile(m_cruiseVelocity.get(), m_acceleration.get(), m_jerk.get());
  }

  /**
   * Applies the profile if any constraint has changed since the last poll. Call this once per loop
   * from {@code periodic()}.
   *
   * @param id unique caller id; pass {@code hashCode()}
   */
  public void poll(int id, Consumer<MotionProfile> apply) {
    LoggedTunableNumber.ifChanged(
        id, () -> apply.accept(get()), m_cruiseVelocity, m_acceleration, m_jerk);
  }
}
