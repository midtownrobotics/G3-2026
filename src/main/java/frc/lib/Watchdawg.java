package frc.lib;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;

public class Watchdawg {
    final String basePath;
    double m_startTime;

    public Watchdawg(Class<?> clazz) {
        basePath = "Watchdog/" + clazz.getSimpleName() + "/";
    }

    public void start() {
        m_startTime = Timer.getFPGATimestamp();
    }

    public void end(String epoch) {
        double totalTime = Timer.getFPGATimestamp() - m_startTime;
        DogLog.log(basePath + epoch, totalTime);
    }
}
