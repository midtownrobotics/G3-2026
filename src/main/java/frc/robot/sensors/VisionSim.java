package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Camera.PoseObservation;

public class VisionSim extends SubsystemBase {
  private final VisionSystemSim m_visionSim = new VisionSystemSim("main");
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final List<PhotonCameraSim> m_cameras;
  private final Supplier<Pose2d> m_poseSupplier;

  public VisionSim(Consumer<PoseObservation> addVisionMeasurement, Supplier<Pose2d> poseSupplier, Camera... cameras) {
    m_visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
    SimCameraProperties cameraProp = new SimCameraProperties();

    m_cameras = List.of(cameras).stream().map(c -> new PhotonCameraSim(c.getCamera(), cameraProp)).toList();

    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;

    for (int i = 0; i < cameras.length; i++) {
      m_visionSim.addCamera(m_cameras.get(i), cameras[i].getTransform());
    }
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_poseSupplier.get());
  }

}
