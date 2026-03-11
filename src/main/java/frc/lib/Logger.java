package frc.lib;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState.RobotMode;
import frc.robot.sensors.Camera.PoseObservation;
import frc.robot.sensors.DetectionCam.DetectionResult;

public class Logger {
  private final String m_basePath;

  public Logger(Class<?> clazz) {
    m_basePath = clazz.getSimpleName() + "/";
  }

  public void log(String name, boolean value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, double value) {
    DogLog.log(m_basePath + name, value);
  }
  
  public void log(String name, int value) {
    DogLog.log(m_basePath + name, value);
  }
  
  public void log(String name, String value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, Distance value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, Angle value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, Pose2d value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, Pose3d value) {
    DogLog.log(m_basePath + name, value);
  }

  public void log(String name, RobotMode value) {
    DogLog.log(m_basePath + name, value);
  }

  public void logPoseObservations(String name, List<PoseObservation> value) {
    Pose3d[] poses = new Pose3d[value.size()];
    double[] timestamps = new double[value.size()];
    int[] tagCounts = new int[value.size()];

    for (int i = 0; i < value.size(); i++) {
      poses[i] = value.get(i).pose();
      timestamps[i] = value.get(i).timestamp();
      tagCounts[i] = value.get(i).tagCount();
    }

    DogLog.log(m_basePath+name+"/poses", poses);
    DogLog.log(m_basePath+name+"/timestamps", timestamps);
    DogLog.log(m_basePath+name+"/tagCounts", tagCounts);
  }

  public void logDetectionResults(String name, List<DetectionResult> value) {
    double[][] allX = new double[value.size()][];
    double[][] allY = new double[value.size()][];

    for (int i = 0; i < value.size(); i++) {
      allX[i] = value.get(i).targetX();
      allY[i] = value.get(i).targetY();
    }

    for (int i = 0; i < value.size(); i++) {
      DogLog.log(m_basePath + name + "/" + i + "/TargetX", allX[i]);
      DogLog.log(m_basePath + name + "/" + i + "/TargetY", allY[i]);
    }
  }
}
