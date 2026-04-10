package frc.robot.sensors;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.FieldConstants;

public class Camera {
  private PhotonCamera m_camera;
  protected Transform3d m_robotToCamera;
  private String m_name;
  private final Alert m_connectionAlert;
  private final double m_stdDevMultiplier;

  public static record PoseObservation(double timestamp, Pose3d pose, int tagCount, double averageDistanceMeters,
      String cameraName, Matrix<N3, N1> standardDevs) {
  }

  public Camera(String name, Transform3d robotToCamera, double stdDevMultiplier) {
    m_name = name;
    m_camera = new PhotonCamera(name);
    m_robotToCamera = robotToCamera;
    m_connectionAlert = new Alert("Camera " + name + " is not connected!", AlertType.kWarning);
    m_stdDevMultiplier = stdDevMultiplier;
  }

  public Camera(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, 4.0);
  }

  public void periodic() {
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
    double base;
    switch (tagCount) {
      case 5:
        base = 0.04;
        break;
      case 4:
        base = 0.04;
        break;
      case 3:
        base = 0.05;
        break;
      default:
        base = 0.07;
        break;
    }
    double distanceMultiplier = Math.max(1.0, avgDistanceMeters / 3.0);
    double stdDev = base * distanceMultiplier * m_stdDevMultiplier;
    return VecBuilder.fill(stdDev, stdDev, 5 * stdDev);
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

      }
    }

    Logger.recordOutput("Vision/" + m_camera.getName() + "/numberOfObservations", observations.size());

    return observations;
  }

  public boolean hasTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();

    return result.hasTargets();
  }

}
