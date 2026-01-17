package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private List<Camera> m_cameras;
  private Consumer<PoseObservation> m_addVisionMeasurement;
  StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/poses", Pose3d.struct).publish();

  public Vision(Consumer<PoseObservation> addVisionMeasurement, Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
  }

  @Override
  public void periodic() {
    for (var camera : m_cameras) {
      for (var observation : camera.getLatestObservations()) {
        m_addVisionMeasurement.accept(observation);
      }
    }

    posePublisher.accept(m_cameras.stream()
        .map((cam) -> cam.getLatestObservations())
        .flatMap(List::stream)
        .map((obs) -> obs.pose()).toArray(Pose3d[]::new));

  }
}
