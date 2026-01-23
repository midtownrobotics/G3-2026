package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
  private PhotonCamera m_camera;
  private PhotonPoseEstimator m_estimator;

  public static record PoseObservation(double timestamp, Pose3d pose, int tagCount) {
  }

  public Camera(String name, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(name);
    m_estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
        robotToCamera);
  }

  public List<PoseObservation> getLatestObservations() {
    return m_camera.getAllUnreadResults().stream()
        .map(m_estimator::estimateCoprocMultiTagPose)
        .flatMap(Optional::stream)
        .map((est) -> new PoseObservation(est.timestampSeconds, est.estimatedPose, est.targetsUsed.size()))
        .toList();
  }
}
