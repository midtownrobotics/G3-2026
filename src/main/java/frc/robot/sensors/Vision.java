package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Watchdawg;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private static final double kMaxDistanceFromFusedPose = 4.0; // meters
  private static final double kCrossCameraAgreementThreshold = 1.0; // meters
  private static final String kTurretCameraName = "Turret";
  private static final double kVisionTimeoutSeconds = 1.1; // extra 0.1s for pipeline latency (observation timestamps lag FPGA)

  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<Pose2d> m_resetPoseConsumer;
  private VisionSystemSim m_visionSim;
  private final Watchdawg m_watchdog;
  private final TimeInterpolatableBuffer<Pose2d> m_acceptedObservations;

  private boolean m_hasVisionUpdate;
  private boolean m_hasAcceptedVisionUpdate;
  private double m_lastAcceptedVisionTimestamp = 0.0;
  private final Trigger m_hasVisionUpdateTrigger;
  private final Trigger m_hasAcceptedVisionUpdateTrigger;
  private final Trigger m_hasRecentAcceptedVisionTrigger;

  public Vision(
      Consumer<PoseObservation> addVisionMeasurement,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPoseConsumer,
      Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;
    m_resetPoseConsumer = resetPoseConsumer;

    m_acceptedObservations = TimeInterpolatableBuffer.createBuffer(0.1);

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));
      m_cameras.forEach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }

    m_hasVisionUpdateTrigger = new Trigger(this::hasVisionUpdate);
    m_hasAcceptedVisionUpdateTrigger = new Trigger(this::hasAcceptedVisionUpdate);
    m_hasRecentAcceptedVisionTrigger = new Trigger(this::hasRecentAcceptedVision);

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
    } else {
      m_hasVisionUpdate = true;
      m_hasAcceptedVisionUpdate = false;
      boolean poseTrusted = hasRecentAcceptedVision();

      // Find turret observation if present
      Translation2d turretTranslation = null;
      for (var obs : observations) {
        if (obs.cameraName().equals(kTurretCameraName)) {
          turretTranslation = obs.pose().toPose2d().getTranslation();
          break;
        }
      }

      for (var observation : observations) {
        Logger.recordOutput("Vision/" + observation.cameraName() + "/observedRobotPose", observation.pose());
        Translation2d obsTranslation = observation.pose().toPose2d().getTranslation();
        double distFromFused = fusedPose.getTranslation().getDistance(obsTranslation);
        Logger.recordOutput("Vision/" + observation.cameraName() + "/distFromFusedPose", distFromFused);

        // Gate 1: reject if far from fused pose (when pose is trusted)
        if (poseTrusted && distFromFused > kMaxDistanceFromFusedPose) {
          Logger.recordOutput("Vision/" + observation.cameraName() + "/rejected", "fused");
          continue;
        }

        // Gate 2: cross-camera rejection (only for non-turret cameras with 2+ observations)
        if (!observation.cameraName().equals(kTurretCameraName) && observations.size() >= 2) {
          boolean agreesWithFused = distFromFused <= kCrossCameraAgreementThreshold;

          if (turretTranslation != null) {
            // Turret available: reject if disagrees with both fused pose and turret
            double distFromTurret = turretTranslation.getDistance(obsTranslation);
            Logger.recordOutput("Vision/" + observation.cameraName() + "/distFromTurret", distFromTurret);
            boolean agreesWithTurret = distFromTurret <= kCrossCameraAgreementThreshold;
            if (!agreesWithFused && !agreesWithTurret) {
              Logger.recordOutput("Vision/" + observation.cameraName() + "/rejected", "turret+fused");
              continue;
            }
          } else {
            // No turret: reject if disagrees with fused pose and not in majority
            if (!agreesWithFused) {
              int agreeing = 0;
              int total = 0;
              for (var other : observations) {
                if (other.cameraName().equals(observation.cameraName())) continue;
                total++;
                if (obsTranslation.getDistance(other.pose().toPose2d().getTranslation())
                    <= kCrossCameraAgreementThreshold) {
                  agreeing++;
                }
              }
              Logger.recordOutput("Vision/" + observation.cameraName() + "/agreeingCameras", agreeing);
              if (agreeing < total / 2.0) {
                Logger.recordOutput("Vision/" + observation.cameraName() + "/rejected", "minority");
                continue;
              }
            }
          }
        }

        Logger.recordOutput("Vision/" + observation.cameraName() + "/rejected", "none");
        m_addVisionMeasurement.accept(observation);
        m_acceptedObservations.addSample(observation.timestamp(), observation.pose().toPose2d());
        m_hasAcceptedVisionUpdate = true;
        m_lastAcceptedVisionTimestamp = observation.timestamp();
      }

      resetRobotPoseIfDiverged(fusedPose);
    }

    Logger.recordOutput("Vision/hasVisionUpdate", m_hasVisionUpdate);
    Logger.recordOutput("Vision/hasAcceptedVisionUpdate", m_hasAcceptedVisionUpdate);
    Logger.recordOutput("Vision/hasRecentAcceptedVision", hasRecentAcceptedVision());

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

  public boolean hasRecentAcceptedVision() {
    return Timer.getFPGATimestamp() - m_lastAcceptedVisionTimestamp < kVisionTimeoutSeconds;
  }

  public Trigger getHasRecentAcceptedVisionTrigger() {
    return m_hasRecentAcceptedVisionTrigger;
  }

  private void resetRobotPoseIfDiverged(Pose2d robotPose) {
    if (!m_acceptedObservations.getInternalBuffer().isEmpty()) {
      Translation2d robotTranslation = robotPose.getTranslation();
      Pose2d latestAcceptedObservationPose = m_acceptedObservations.getInternalBuffer().lastEntry().getValue();
      Logger.recordOutput("Vision/acceptedObservationsLastEntry", latestAcceptedObservationPose);
      if (robotTranslation.getDistance(latestAcceptedObservationPose.getTranslation()) > 1.5
          && Timer.getFPGATimestamp() - m_acceptedObservations.getInternalBuffer().lastKey() < 0.04) {
        Logger.recordOutput("Vision/resetPose", true);
        m_resetPoseConsumer.accept(latestAcceptedObservationPose);
        return;
      }
    }
    Logger.recordOutput("Vision/resetPose", false);
  }
}
