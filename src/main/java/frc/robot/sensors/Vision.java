package frc.robot.sensors;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Watchdawg;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private VisionSystemSim m_visionSim;
  private final Watchdawg m_watchdog;
  private final TimeInterpolatableBuffer<Pose2d> m_observations;

  public Vision(
      Consumer<PoseObservation> addVisionMeasurement,
      Supplier<Pose2d> poseSupplier,
      Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
      m_cameras.forEach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }

    m_observations = TimeInterpolatableBuffer.createBuffer(0.1);

    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    List<PoseObservation> observations = m_cameras.stream().flatMap(c -> c.getLatestObservations().stream()).toList();

    if (observations.isEmpty()) {
      m_watchdog.end("periodic");
      return;
    }

    observations.forEach(o -> m_observations.addSample(o.timestamp(), o.pose().toPose2d()));

    double meanX = m_observations.getInternalBuffer().values().stream().mapToDouble(p -> p.getX()).average().orElse(0);
    double meanY = m_observations.getInternalBuffer().values().stream().mapToDouble(p -> p.getY()).average().orElse(0);

    for (var camera : m_cameras) {
      camera.periodic();
      Logger.recordOutput("Vision/cameraPoses/" + camera.getName(),
          new Pose3d(m_poseSupplier.get()).transformBy(camera.getRobotToCamera()));
    }

    for (var observation : observations) {
      Logger.recordOutput("Vision/" + observation.cameraName() + "/observedPose", observation.pose());
      if (Math.abs(observation.pose().getX() - meanX) > 1 || Math.abs(observation.pose().getY() - meanY) > 1) {
        continue;
      }
      m_addVisionMeasurement.accept(observation);
    }
    m_watchdog.end("periodic");
  }

  public Optional<Pose2d> getPoseAtTime(double time) {
    return m_observations.getSample(time);
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_poseSupplier.get());
  }
}
