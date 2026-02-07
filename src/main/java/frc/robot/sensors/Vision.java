package frc.robot.sensors;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private final AprilTagFieldLayout m_fieldLayout;
  private VisionSystemSim m_visionSim;

  StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/RobotPoses", Pose3d.struct).publish();

  StructArrayPublisher<Pose3d> tagPosePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/TagPoses", Pose3d.struct).publish();

  public Vision(Consumer<PoseObservation> addVisionMeasurement, Supplier<Pose2d> poseSupplier, Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    DogLog.log("Vision/CameraPoses/", m_cameras.stream().map(c -> new Pose3d(m_poseSupplier.get()).transformBy(c.getRobotToCamera())).toArray(Pose3d[]::new));

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_cameras.forEach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }
  }

  public List<Camera> getCameras() {
    return m_cameras;
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

    List<Pose3d> visibleTagPoses = m_cameras.stream()
        .flatMap((cam) -> cam.getTargetsInView().stream())
        .distinct()
        .map((tagId) -> m_fieldLayout.getTagPose(tagId))
        .filter(opt -> opt.isPresent())
        .map(opt -> opt.get())
        .toList();

    tagPosePublisher.accept(visibleTagPoses.toArray(new Pose3d[0]));
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_poseSupplier.get());
  }
}
