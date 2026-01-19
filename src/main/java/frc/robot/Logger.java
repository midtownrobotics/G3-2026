package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Logger {
  private static HashMap<String, StructPublisher<Pose2d>> pose2dPublishers = new HashMap<>();
  private static HashMap<String, DoublePublisher> doublePublishers = new HashMap<>();
  private static HashMap<String, StructPublisher<Pose3d>> pose3dPublishers = new HashMap<>();
  private static HashMap<String, StructArrayPublisher<Pose3d>> pose3dArrayPublishers = new HashMap<>();

  public static void log(String name, double value) {
    if (!doublePublishers.containsKey(name)) {
      doublePublishers.put(name, NetworkTableInstance.getDefault().getDoubleTopic(name).publish());
    }

    doublePublishers.get(name).accept(value);
  }

  public static void logPose2d(String name, Pose2d pose) {
    if (!pose2dPublishers.containsKey(name)) {
      pose2dPublishers.put(name, NetworkTableInstance.getDefault().getStructTopic(name, Pose2d.struct).publish());
    }

    pose2dPublishers.get(name).accept(pose);
  }

  public static void logPose3d(String name, Pose3d pose) {
    if (!pose3dPublishers.containsKey(name)) {
      pose3dPublishers.put(name, NetworkTableInstance.getDefault().getStructTopic(name, Pose3d.struct).publish());
    }

    pose3dPublishers.get(name).accept(pose);
  }

  public static void logPose3dArray(String name, Pose3d... poses) {
    if (!pose3dArrayPublishers.containsKey(name)) {
      pose3dArrayPublishers.put(
          name,
          NetworkTableInstance.getDefault().getStructArrayTopic(name, Pose3d.struct).publish());
    }

    pose3dArrayPublishers.get(name).accept(poses);
  }
}
