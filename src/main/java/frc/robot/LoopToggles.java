package frc.robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Dashboard switches for skipping pieces of the main loop, so loop overruns can be bisected by
 * turning things off one at a time and watching {@code Watchdog/_Summary/robotPeriodicTotal}.
 *
 * <p>Toggles are <b>latched</b>: the NetworkTables value is only sampled while the driver station is
 * disabled. Flipping a switch mid-match therefore does nothing until the next disable, which keeps
 * someone from freezing odometry (and with it auto-aim, shoot-on-the-move, and path following) while
 * the robot is moving.
 */
public final class LoopToggles {
  private static final Map<String, LoopToggle> s_toggles = new LinkedHashMap<>();

  private static final Alert s_disabledAlert = new Alert("", AlertType.kWarning);

  public static final class LoopToggle {
    private final String m_name;
    private final LoggedNetworkBoolean m_nt;
    private boolean m_latched = true;

    private LoopToggle(String name) {
      m_name = name;
      m_nt = new LoggedNetworkBoolean("Toggles/Enabled/" + name, true);
    }

    /** Whether the gated code should run this loop. */
    public boolean get() {
      return m_latched;
    }

    public String getName() {
      return m_name;
    }
  }

  private LoopToggles() {
  }

  /** Creates a toggle, defaulted on, published at {@code Toggles/Enabled/<name>}. */
  public static LoopToggle create(String name) {
    return s_toggles.computeIfAbsent(name, LoopToggle::new);
  }

  /** Call once per loop, before anything the toggles gate. */
  public static void periodic() {
    boolean canSample = DriverStation.isDisabled();
    List<String> disabled = new ArrayList<>();

    for (LoopToggle toggle : s_toggles.values()) {
      if (canSample) {
        toggle.m_latched = toggle.m_nt.get();
      }

      Logger.recordOutput("Toggles/Latched/" + toggle.m_name, toggle.m_latched);

      if (!toggle.m_latched) {
        disabled.add(toggle.m_name);
      }
    }

    s_disabledAlert.setText("Loop toggles OFF: " + String.join(", ", disabled));
    s_disabledAlert.set(!disabled.isEmpty());
  }
}
