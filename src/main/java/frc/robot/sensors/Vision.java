package frc.robot.sensors;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Watchdawg;
import frc.robot.Robot;
import frc.robot.sensors.Camera.PoseObservation;

public class Vision extends SubsystemBase {
  private static final double kMaxDistanceFromFusedPose = 4.0; // meters
  private static final double kVisionTimeoutSeconds = 1.0;

  private final List<Camera> m_cameras;
  private final Consumer<PoseObservation> m_addVisionMeasurement;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Consumer<Pose2d> m_resetPoseConsumer;
  private VisionSystemSim m_visionSim;
  private final Watchdawg m_watchdog;
  private final TimeInterpolatableBuffer<Pose2d> m_acceptedObservations;

  private final LoggedNetworkBoolean m_enableVisionObservations = new LoggedNetworkBoolean(
      "Toggles/UseVisionObservations", true);

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
      // m_cameras.forE\ach(c -> m_visionSim.addCamera(c.getSimCamera(), c.getRobotToCamera()));
    }

    m_hasVisionUpdateTrigger = new Trigger(this::hasVisionUpdate);
    m_hasAcceptedVisionUpdateTrigger = new Trigger(this::hasAcceptedVisionUpdate);
    m_hasRecentAcceptedVisionTrigger = new Trigger(this::hasRecentAcceptedVision);

    m_watchdog = new Watchdawg(getClass());

    SmartDashboard.putData("Commands/Vision/ResetRobotPoseToLatestVisionPose",
        resetRobotPoseToLatestVisionPoseCommand());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    Pose2d robotPose = m_poseSupplier.get();
    Pose3d robotPose3d = new Pose3d(robotPose);

    for (var camera : m_cameras) {
      camera.periodic();
      Logger.recordOutput("Vision/" + camera.getName() + "/cameraPose",
          robotPose3d.transformBy(camera.getRobotToCamera()));
    }

    List<PoseObservation> observations = m_cameras.stream().filter(c -> c.isEnabled())
        .flatMap(c -> c.getLatestObservations().stream()).sorted((a, b) -> Double.compare(a.timestamp(), b.timestamp()))
        .toList();

    Logger.recordOutput("Vision/observationsSize", observations.size());

    if (observations.isEmpty()) {
      m_hasVisionUpdate = false;
      m_hasAcceptedVisionUpdate = false;
    } else {
      m_hasVisionUpdate = true;
      m_hasAcceptedVisionUpdate = false;
      boolean poseTrusted = hasRecentAcceptedVision();

      for (var observation : observations) {
        Logger.recordOutput("Vision/" + observation.cameraName() + "/observedRobotPose", observation.pose());
        double distFromFused = robotPose.getTranslation()
            .getDistance(observation.pose().toPose2d().getTranslation());
        Logger.recordOutput("Vision/" + observation.cameraName() + "/distFromFusedPose", distFromFused);

        // if (poseTrusted && distFromFused > kMaxDistanceFromFusedPose) {
        //   continue;
        // }

        m_acceptedObservations.addSample(observation.timestamp(), observation.pose().toPose2d());
        m_hasAcceptedVisionUpdate = true;
        m_lastAcceptedVisionTimestamp = Timer.getFPGATimestamp();

        if (m_enableVisionObservations.get()) {
          m_addVisionMeasurement.accept(observation);
        }
      }
    }

    Logger.recordOutput("Vision/hasVisionUpdate", m_hasVisionUpdate);
    Logger.recordOutput("Vision/hasAcceptedVisionUpdate", m_hasAcceptedVisionUpdate);
    Logger.recordOutput("Vision/hasRecentAcceptedVision", hasRecentAcceptedVision());
		Logger.recordOutput("Vision/arePipelinesReady", areAllPipelinesReady());

    m_watchdog.end("periodic");
  }

  public Optional<Pose2d> getPoseAtTime(double time) {
    return m_acceptedObservations.getSample(time);
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

  private void resetRobotPoseToVision() {
    Pose2d latestAcceptedObservationPose = m_acceptedObservations.getInternalBuffer().lastEntry().getValue();
    m_resetPoseConsumer.accept(latestAcceptedObservationPose);
  }

  public Command resetRobotPoseToLatestVisionPoseCommand() {
    return Commands.runOnce(this::resetRobotPoseToVision);
  }

	public boolean areAllPipelinesReady() {
		return m_cameras.stream().allMatch(c -> c.getCamera().getPipelineIndex() == 0);
	}

	public void setPipelinesToIndex(int index) {
			m_cameras.forEach(c -> c.getCamera().setPipelineIndex(index));
	}
}
