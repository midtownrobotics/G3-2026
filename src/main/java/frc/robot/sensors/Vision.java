package frc.robot.sensors;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Watchdawg;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<Pose2d> m_resetPoseConsumer;
  private VisionSystemSim m_visionSim;
  private final Watchdawg m_watchdog;
  private final TimeInterpolatableBuffer<Pose2d> m_observations;
  private final TimeInterpolatableBuffer<Pose2d> m_acceptedObservations;

  private final List<String> m_cameraHierarchy = List.of(
      "Turret",
      "Rear",
      "Rear Left",
      "Front Left",
      "Rear Right");

  private boolean m_hasVisionUpdate;
  private final Trigger m_hasVisionUpdateTrigger;

  public Vision(
      Consumer<PoseObservation> addVisionMeasurement,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPoseConsumer,
      Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;
    m_resetPoseConsumer = resetPoseConsumer;

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
      // m_cameras.forEach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }

    m_observations = TimeInterpolatableBuffer.createBuffer(0.1);
    m_acceptedObservations = TimeInterpolatableBuffer.createBuffer(0.1);

    m_hasVisionUpdateTrigger = new Trigger(this::hasVisionUpdate);

    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    Pose3d robotPose = new Pose3d(m_poseSupplier.get());

    for (var camera : m_cameras) {
      camera.periodic();
      Logger.recordOutput("Vision/" + camera.getName() + "/cameraPose",
          robotPose.transformBy(camera.getRobotToCamera()));
    }

    List<PoseObservation> observations = m_cameras.stream().flatMap(c -> c.getLatestObservations().stream()).toList();

    String desiredCamera = getDesiredCameraName(observations);

    Logger.recordOutput("Vision/desiredCamera", desiredCamera);

    observations = observations.stream().filter(o -> o.cameraName().equals(desiredCamera)).toList();

    Logger.recordOutput("Vision/observationsSize", observations.size());

    if (observations.isEmpty()) {
      m_hasVisionUpdate = false;
      m_watchdog.end("periodic");
      return;
    }

    m_hasVisionUpdate = true;

    observations.forEach(o -> m_observations.addSample(o.timestamp(), o.pose().toPose2d()));

    double meanX = m_observations.getInternalBuffer().values().stream().mapToDouble(p -> p.getX()).average().orElse(0);
    double meanY = m_observations.getInternalBuffer().values().stream().mapToDouble(p -> p.getY()).average().orElse(0);

    for (var observation : observations) {
      Logger.recordOutput("Vision/" + observation.cameraName() + "/observedRobotPose", observation.pose());
      if (Math.abs(observation.pose().getX() - meanX) > 1 || Math.abs(observation.pose().getY() - meanY) > 1) {
        continue;
      }
      m_acceptedObservations.addSample(observation.timestamp(), observation.pose().toPose2d());
      m_addVisionMeasurement.accept(observation);
    }

    resetRobotPoseIfDiverged(robotPose.toPose2d());

    Logger.recordOutput("Vision/hasVisionUpdate", m_hasVisionUpdateTrigger.getAsBoolean());

    m_watchdog.end("periodic");
  }

  private String getDesiredCameraName(List<PoseObservation> observations) {
    Set<String> camerasWithObservations = observations.stream().map((o) -> o.cameraName()).collect(Collectors.toSet());

    for (String cameraName : m_cameraHierarchy) {
      if (camerasWithObservations.contains(cameraName)) {
        return cameraName;
      }
    }

    return null;
  }

  private void resetRobotPoseIfDiverged(Pose2d robotPose) {
    if (!m_acceptedObservations.getInternalBuffer().isEmpty()) {
      Translation2d robotTranslation = robotPose.getTranslation();
      Pose2d latestAcceptedObservationPose = m_acceptedObservations.getInternalBuffer().lastEntry().getValue();
      Logger.recordOutput("Vision/acceptedObservationsLastEntry", latestAcceptedObservationPose);

      if (robotTranslation.getDistance(latestAcceptedObservationPose.getTranslation()) > 0.5
          && Timer.getFPGATimestamp() - m_acceptedObservations.getInternalBuffer().lastKey() < 0.04) {
        Logger.recordOutput("Vision/resetPose", true);
        m_resetPoseConsumer.accept(latestAcceptedObservationPose);
        return;
      }
    }
    Logger.recordOutput("Vision/resetPose", false);
  }

  public Optional<Pose2d> getPoseAtTime(double time) {
    return m_acceptedObservations.getSample(time);
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_poseSupplier.get());
  }

  public boolean hasVisionUpdate() {
    return m_hasVisionUpdate;
  }

  public Trigger getHasVisionUpdateTrigger() {
    return m_hasVisionUpdateTrigger;
  }
}
