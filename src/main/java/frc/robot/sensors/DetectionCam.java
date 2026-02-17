package frc.robot.sensors;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DetectionCam {
  private PhotonCamera m_camera;

  private static final double FUEL_CAM_FOV_HORIZONTAL = 60.0; // degrees

  public static final double FUEL_DIAMETER_METERS = 0.150114; // 5.91 inches
  public static final double FOCAL_LENGTH_PIXELS = 500.0;

  public DetectionCam(String name) {
    m_camera = new PhotonCamera(name);
  }

  public PhotonCamera getCamera() {
    return m_camera;
  }

  public PhotonCameraSim getSimCamera() {
    SimCameraProperties properties = new SimCameraProperties();
    return new PhotonCameraSim(this.getCamera(), properties);
  }

  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  public double getFuelX() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double normalizedX = yaw / (FUEL_CAM_FOV_HORIZONTAL / 2.0);
      return Math.max(-1.0, Math.min(1.0, normalizedX));
    }
    return 0.0;
  }

  public boolean hasTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    return result.hasTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget();
    }
    return null;
  }

  public double getTargetYaw() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget().getYaw();
    }
    return 0.0;
  }

  public double getTargetPitch() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getBestTarget().getPitch();
    }
    return 0.0;
  }
}
