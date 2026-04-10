package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Watchdawg;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private static final double kMaxDistanceFromFusedPose = 4.0; // meters

  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private VisionSystemSim m_visionSim;
  private final Watchdawg m_watchdog;

  private boolean m_hasVisionUpdate;
  private boolean m_hasAcceptedVisionUpdate;
  private final Trigger m_hasVisionUpdateTrigger;
  private final Trigger m_hasAcceptedVisionUpdateTrigger;

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

    m_hasVisionUpdateTrigger = new Trigger(this::hasVisionUpdate);
    m_hasAcceptedVisionUpdateTrigger = new Trigger(this::hasAcceptedVisionUpdate);

    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    Pose2d fusedPose = m_poseSupplier.get();
    Pose3d robotPose = new Pose3d(fusedPose);

    for (var camera : m_cameras) {
      camera.periodic();
      Logger.recordOutput("Vision/" + camera.getName() + "/cameraPose",
          robotPose.transformBy(camera.getRobotToCamera()));
    }

    List<PoseObservation> observations = m_cameras.stream().flatMap(c -> c.getLatestObservations().stream()).toList();

    Logger.recordOutput("Vision/observationsSize", observations.size());

    if (observations.isEmpty()) {
      m_hasVisionUpdate = false;
      m_hasAcceptedVisionUpdate = false;
      m_watchdog.end("periodic");
      return;
    }

    m_hasVisionUpdate = true;
    m_hasAcceptedVisionUpdate = false;

    for (var observation : observations) {
      Logger.recordOutput("Vision/" + observation.cameraName() + "/observedRobotPose", observation.pose());
      double distFromFused = fusedPose.getTranslation()
          .getDistance(observation.pose().toPose2d().getTranslation());
      Logger.recordOutput("Vision/" + observation.cameraName() + "/distFromFusedPose", distFromFused);
      if (distFromFused > kMaxDistanceFromFusedPose) {
        continue;
      }
      m_addVisionMeasurement.accept(observation);
      m_hasAcceptedVisionUpdate = true;
    }

    Logger.recordOutput("Vision/hasVisionUpdate", m_hasVisionUpdateTrigger.getAsBoolean());
    Logger.recordOutput("Vision/hasAcceptedVisionUpdate", m_hasAcceptedVisionUpdateTrigger.getAsBoolean());

    m_watchdog.end("periodic");
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

  public boolean hasAcceptedVisionUpdate() {
    return m_hasAcceptedVisionUpdate;
  }

  public Trigger getHasAcceptedVisionUpdateTrigger() {
    return m_hasAcceptedVisionUpdateTrigger;
  }
}
