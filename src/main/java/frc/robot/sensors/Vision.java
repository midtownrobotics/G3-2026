package frc.robot.sensors;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggerUtil;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private VisionSystemSim m_visionSim;

  StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Vision/poses", Pose3d.struct).publish();

  public Vision(Consumer<PoseObservation> addVisionMeasurement, Supplier<Pose2d> poseSupplier, Camera... cameras) {
    m_cameras = List.of(cameras);
    m_addVisionMeasurement = addVisionMeasurement;
    m_poseSupplier = poseSupplier;

    if (Robot.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_cameras.forEach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }
  }

  @Override
  public void periodic() {
    for (var camera : m_cameras) {
      LoggerUtil.log("cameraPoses/" + camera.getName(), new Pose3d(m_poseSupplier.get()).transformBy(camera.getRobotToCamera()));
      for (var observation : camera.getLatestObservations()) {
        LoggerUtil.log("observedPose", observation.pose());
        m_addVisionMeasurement.accept(observation);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    m_visionSim.update(m_poseSupplier.get());
  }
}
