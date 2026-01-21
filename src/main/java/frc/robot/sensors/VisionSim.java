package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.sensors.Camera.PoseObservation;

public class VisionSim {
  private final VisionSystemSim m_visionSim = new VisionSystemSim("main");
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final List<PhotonCameraSim> m_cameras;

  public VisionSim(Consumer<PoseObservation> addVisionMeasurement, Camera... cameras) {
    m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
    SimCameraProperties cameraProp = new SimCameraProperties();

    m_cameras = List.of(cameras).stream().map(c -> new PhotonCameraSim(c.getCamera(), cameraProp)).toList();

    m_addVisionMeasurement = addVisionMeasurement;

    for (int i = 0; i < cameras.length; i++) {
      m_visionSim.addCamera(m_cameras.get(i), cameras[i].getTransform());
    }

  }
}
