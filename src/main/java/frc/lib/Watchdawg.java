package frc.lib;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Watchdawg {
  final String basePath;
  private double m_startTime;

  public Watchdawg(Class<?> clazz) {
    basePath = "Watchdog/" + clazz.getSimpleName() + "/";
    m_startTime = 0;
  }

  public void start() {
    m_startTime = Timer.getFPGATimestamp();
  }

  public void end(String epoch) {
    double totalTime = Timer.getFPGATimestamp() - m_startTime;
    Logger.recordOutput(basePath + epoch, totalTime);
  }
}
