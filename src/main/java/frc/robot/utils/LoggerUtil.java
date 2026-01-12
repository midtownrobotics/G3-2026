package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

public class LoggerUtil {

    /**
     * Util method to log cycle time breakdowns
     */
    public static void recordLatencyOutput(String name, double start, double end) {
        Logger.recordOutput("Logger/UserCode/" + name, (end - start) / 1000);
    }

}
