package frc.lib;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.sensors.Camera.PoseObservation;

public class LoggerUtil {
  public static void log(String name, boolean value) {
    DogLog.log(getPath(name), value);
  }

  public static void log(String name, double value) {
    DogLog.log(getPath(name), value);
  }

  public static void log(String name, int value) {
    DogLog.log(getPath(name), value);
  }

  public static void log(String name, String value) {
    DogLog.log(getPath(name), value);
  }

  public static void log(String name, Distance value) {
    DogLog.log(getPath(name), value);
  }

  public static void log(String name, List<PoseObservation> observations) {
    String path = getPath(name);

    Pose3d[] poses = new Pose3d[observations.size()];
    double[] timestamps = new double[observations.size()];
    int[] tagCounts = new int[observations.size()];

    for (int i = 0; i < observations.size(); i++) {
      poses[i] = observations.get(i).pose();
      timestamps[i] = observations.get(i).timestamp();
      tagCounts[i] = observations.get(i).tagCount();
    }

    DogLog.log(path + "/Poses", poses);
    DogLog.log(path + "/Timestamps", timestamps);
    DogLog.log(path + "/TagCounts", tagCounts);
  }

  private static String getPath(String name) {
    String className = StackWalker.getInstance()
        .walk(frames -> frames.skip(2).findFirst())
        .get()
        .getClassName();
    String simpleName = className.substring(className.lastIndexOf('.') + 1);
    return simpleName + "/" + name;
  }
}
