package frc.lib;

import java.util.LinkedHashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

/**
 * Lightweight stopwatch for finding loop overruns. Each instance publishes its epochs under
 * {@code Watchdog/<SimpleClassName>/<epoch>}.
 *
 * <p>Three ways to record an epoch:
 *
 * <ul>
 *   <li>{@link #end(String)} — elapsed since {@link #start()}, mark untouched.
 *   <li>{@link #lap(String)} — elapsed since {@link #start()} or the previous lap, then resets the
 *       mark. Use this to time several sequential blocks without repeating {@code start()}.
 *   <li>{@link #total(String)} — elapsed since {@link #start()}, for a whole-body total alongside
 *       per-block laps.
 * </ul>
 *
 * <p>Every recorded epoch is also kept in a static registry so derived numbers can be computed
 * without re-deriving them by hand in AdvantageScope. See {@link #last(Class, String)} and
 * {@link #sumPeriodics(Class...)}.
 */
public class Watchdawg {
  private static final String kSummaryPath = "Watchdog/_Summary/";

  /** Most recent duration of an epoch, plus the loop it was recorded on. */
  private static record Epoch(double duration, long loop) {
  }

  private static final Map<String, Epoch> s_epochs = new LinkedHashMap<>();

  private static long s_loop = 0;
  private static double s_lastLoopStart = 0.0;
  private static double s_lastLoopEnd = 0.0;

  final String basePath;
  private double m_startTime;
  private double m_markTime;

  public Watchdawg(Class<?> clazz) {
    basePath = "Watchdog/" + clazz.getSimpleName() + "/";
    m_startTime = 0;
    m_markTime = 0;
  }

  public void start() {
    m_startTime = Timer.getFPGATimestamp();
    m_markTime = m_startTime;
  }

  /** Records elapsed time since {@link #start()}. Does not move the lap mark. */
  public void end(String epoch) {
    record(epoch, Timer.getFPGATimestamp() - m_markTime);
  }

  /** Records elapsed time since {@link #start()} or the previous lap, then resets the lap mark. */
  public void lap(String epoch) {
    double now = Timer.getFPGATimestamp();
    record(epoch, now - m_markTime);
    m_markTime = now;
  }

  /** Records elapsed time since {@link #start()}, ignoring any laps taken in between. */
  public void total(String epoch) {
    record(epoch, Timer.getFPGATimestamp() - m_startTime);
  }

  /** Records an already-measured duration under this watchdog's path. */
  public void record(String epoch, double duration) {
    Logger.recordOutput(basePath + epoch, duration);
    s_epochs.put(basePath + epoch, new Epoch(duration, s_loop));
  }

  /**
   * Call once at the very top of {@code robotPeriodic()}. Advances the loop counter used by
   * {@link #last(Class, String)} to reject stale values, and publishes the time spent outside the
   * user loop.
   */
  public static void newLoop() {
    double now = Timer.getFPGATimestamp();
    s_loop++;

    if (s_lastLoopEnd > 0.0) {
      // Everything between the end of the last robotPeriodic and the start of this one: the
      // AdvantageKit logger's before/after-user work, WPILOG writes, NT4 flush, GC, and the idle
      // sleep. When the loop overruns the sleep vanishes and this is pure overhead.
      Logger.recordOutput(kSummaryPath + "frameworkOverhead", now - s_lastLoopEnd);
      Logger.recordOutput(kSummaryPath + "loopPeriod", now - s_lastLoopStart);
    }

    s_lastLoopStart = now;
  }

  /** Call once at the very bottom of {@code robotPeriodic()}. */
  public static void endLoop() {
    s_lastLoopEnd = Timer.getFPGATimestamp();
  }

  /**
   * Most recent duration of an epoch, or 0.0 if it was not recorded on the current loop. The
   * staleness check matters: a subsystem whose periodic is toggled off stops recording, and a stale
   * value would silently corrupt any derived sum.
   */
  public static double last(Class<?> clazz, String epoch) {
    Epoch entry = s_epochs.get("Watchdog/" + clazz.getSimpleName() + "/" + epoch);
    if (entry == null || entry.loop() != s_loop) {
      return 0.0;
    }
    return entry.duration();
  }

  /** Sums this loop's {@code "periodic"} epoch across the given classes. */
  public static double sumPeriodics(Class<?>... classes) {
    double sum = 0.0;
    for (Class<?> clazz : classes) {
      sum += last(clazz, "periodic");
    }
    return sum;
  }
}
