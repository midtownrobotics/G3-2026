package frc.robot.sensors;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.LoggedTunableNumber;
import frc.robot.constants.FieldConstants;

public class Camera {
  private static final double kAmbiguityThreshold = 0.4;

  private PhotonCamera m_camera;
  protected Transform3d m_robotToCamera;
  private String m_name;
  private final Alert m_connectionAlert;
  private final double m_stdDevMultiplier;
  private final Supplier<Boolean> m_enabledSupplier;

  private static final double kDefaultStdMultiplier = 2;

  private static final LoggedTunableNumber kCameraStdDevMultiplier = new LoggedTunableNumber(
      "Vision/CameraStdDevMultiplier", 1.0);

  public static record PoseObservation(double timestamp, Pose3d pose, int tagCount, double averageDistanceMeters,
      String cameraName, Matrix<N3, N1> standardDevs) {
  }

  public Camera(String name, Transform3d robotToCamera, double stdDevMultiplier, Supplier<Boolean> enabledSupplier) {
    m_name = name;
    m_camera = new PhotonCamera(name);
    m_robotToCamera = robotToCamera;
    m_connectionAlert = new Alert("Camera " + name + " is not connected!", AlertType.kWarning);
    m_stdDevMultiplier = stdDevMultiplier;
    m_enabledSupplier = enabledSupplier;
  }

  public Camera(String name, Transform3d robotToCamera, Supplier<Boolean> enabledSupplier) {
    this(name, robotToCamera, kDefaultStdMultiplier, enabledSupplier);
  }

  public Camera(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, kDefaultStdMultiplier, () -> true);
  }

  public void periodic() {
    Logger.recordOutput("Vision/" + m_camera.getName() + "/enabled", m_enabledSupplier.get());
    Logger.recordOutput("Vision/" + m_camera.getName() + "/connected", m_camera.isConnected());
    m_connectionAlert.set(!m_camera.isConnected());
  }

  public String getName() {
    return m_name;
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  public PhotonCamera getCamera() {
    return m_camera;
  }

  public PhotonCameraSim getSimCamera() {
    SimCameraProperties properties = new SimCameraProperties();
    return new PhotonCameraSim(this.getCamera(), properties);
  }

  private Matrix<N3, N1> calculateStandardDevs(int tagCount, double avgDistanceMeters) {
    double distanceMultiplier = Math.pow(avgDistanceMeters, 1.5);
    double stdDev = distanceMultiplier / tagCount * m_stdDevMultiplier * kCameraStdDevMultiplier.get();
    return VecBuilder.fill(stdDev, stdDev, stdDev);
  }

  public List<PoseObservation> getLatestObservations() {
    List<PoseObservation> observations = new LinkedList<>();

    for (var result : m_camera.getAllUnreadResults()) {
      Logger.recordOutput("Vision/" + m_camera.getName() + "/timeStamp", result.getTimestampSeconds());
      if (result.multitagResult.isPresent()) {
        List<Pose3d> tagPoses = result.targets.stream().map(t -> t.getFiducialId())
            .map(d -> FieldConstants.kTagLayout.getTagPose(d).get()).toList();
        Logger.recordOutput("Vision/" + m_camera.getName() + "/isSingleTagResult", false);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/tagPoses", tagPoses.toArray(Pose3d[]::new));
        var multitagResult = result.multitagResult.get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(getRobotToCamera().inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double avgDistance = tagPoses.stream().map(Pose3d::getTranslation)
            .mapToDouble(p -> robotPose.getTranslation().getDistance(p)).average().orElse(Double.MAX_VALUE);

        if (robotPose.getMeasureZ().abs(Feet) > 1.5) {
          continue;
        }

        if (avgDistance > Feet.of(25).in(Meters)) {
          continue;
        }

        int tagCount = multitagResult.fiducialIDsUsed.size();
        observations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                tagCount,
                avgDistance,
                m_name,
                calculateStandardDevs(tagCount, avgDistance)));

      } else if (result.hasTargets()) {
        // Single-tag fallback: disambiguate two PnP solutions using estimated heading.
        // Currently log-only — not added to observations.
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null || bestTarget.getFiducialId() < 0) {
          continue;
        }

        int tagId = bestTarget.getFiducialId();
        double ambiguity = bestTarget.getPoseAmbiguity();
        double areaPercent = bestTarget.getArea();

        Optional<Pose3d> tagFieldPose = FieldConstants.kTagLayout.getTagPose(tagId);
        if (tagFieldPose.isEmpty()) {
          continue;
        }

        Transform3d cameraToRobot = getRobotToCamera().inverse();
        Pose3d robotPose0 = tagFieldPose.get()
            .transformBy(bestTarget.getBestCameraToTarget().inverse())
            .transformBy(cameraToRobot);
        Pose3d robotPose1 = tagFieldPose.get()
            .transformBy(bestTarget.getAlternateCameraToTarget().inverse())
            .transformBy(cameraToRobot);

        Logger.recordOutput("Vision/" + m_camera.getName() + "/isSingleTagResult", true);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagId", tagId);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagAmbiguity", ambiguity);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagAreaPercent", areaPercent);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagPose0", robotPose0);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagPose1", robotPose1);

        if (ambiguity < 0 || ambiguity > kAmbiguityThreshold) {
          Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagRejection", "ambiguity");
          continue;
        }

        Pose3d robotPose = robotPose0;
        double distance = robotPose.getTranslation().getDistance(tagFieldPose.get().getTranslation());

        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagDistance", distance);
        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagPose", robotPose);

        if (distance > 4.0) {
          Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagRejection", "distance");
          continue;
        }

        if (areaPercent < 0.6) {
          Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagRejection", "area");
          continue;
        }

        if (robotPose.getMeasureZ().abs(Feet) > 1.5) {
          Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagRejection", "z-height");
          continue;
        }

        Logger.recordOutput("Vision/" + m_camera.getName() + "/singleTagRejection", "none");
      }
    }

    Logger.recordOutput("Vision/" + m_camera.getName() + "/numberOfObservations", observations.size());

    return observations;
  }

  public boolean hasTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();

    return result.hasTargets();
  }

  public boolean isEnabled() {
    return m_enabledSupplier.get();
  }

  public boolean isConnected() {
    return m_camera.isConnected();
  }

}
