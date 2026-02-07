package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_estimator;
  private Transform3d m_robotToCamera;
  private String m_name;

  public static record PoseObservation(double timestamp, Pose3d pose, int tagCount) {
  }

  public Camera(String name, Transform3d robotToCamera) {
    m_name = name;
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        robotToCamera);

    m_robotToCamera = robotToCamera;
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

  public List<Integer> getTargetsInView() {
    return (
      getCamera()
        .getAllUnreadResults()
        .stream()
        .flatMap((est) -> 
          est.getTargets()
            .stream()
            .map((t) ->
                t.getFiducialId()
            )
        )
    ).toList();
  }

  public List<PoseObservation> getLatestObservations() {
    return m_camera.getAllUnreadResults().stream()
        .map(m_estimator::estimateCoprocMultiTagPose)
        .flatMap(Optional::stream)
        .map((est) -> new PoseObservation(est.timestampSeconds, est.estimatedPose, est.targetsUsed.size()))
        .toList();
  }
}
