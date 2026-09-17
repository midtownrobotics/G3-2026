package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Watchdawg;

/**
 * Camera mounting-offset characterization against a wall of AprilTags at known relative positions.
 *
 * <p>There is no objective ground truth for robot pose, so this works off <em>disagreement</em>:
 * each camera independently produces a field-to-camera pose, and each implies a robot pose via its
 * own robot-to-camera transform. If every offset were correct, all cameras would imply the same
 * robot pose at the same instant. Any consistent difference is a direct measurement of offset
 * error.
 *
 * <p>Two things are provided:
 *
 * <ul>
 *   <li><b>Live pairwise disagreement</b> (this class's {@code periodic}) under
 *       {@code VisionCal/Pairs/&lt;A&gt;-&lt;B&gt;/}. The running <i>mean</i> is bias, i.e. real
 *       offset error; the running <i>stddev</i> is per-frame noise. A mean well above the stddev is
 *       the signal that a transform is actually wrong.
 *   <li><b>Station capture</b>, which labels windows of the log so the offline solver in
 *       {@code tools/vision_cal/solve_offsets.py} can fit per-camera 6-DOF corrections.
 * </ul>
 *
 * <p>Only <em>relative</em> corrections are recoverable this way: an error common to every camera is
 * invisible, and the solver gauge-fixes one camera as the reference.
 *
 * <p>Statistics accumulate only while the robot is essentially stationary. Latency and timestamp
 * error also produce disagreement, but that kind scales with velocity and would otherwise be
 * misread as a mounting error.
 */
public class VisionCalibration extends SubsystemBase {
  /** Two results must be at least this close in time to be compared against each other. */
  private static final double kMaxPairTimeSkewSeconds = 0.04;

  /** Above this chassis speed, disagreement is dominated by latency rather than mounting error. */
  private static final double kStationarySpeedMetersPerSec = 0.05;
  private static final double kStationaryOmegaRadPerSec = 0.05;

  /** How long a station capture holds, and therefore how many frames it labels. */
  private static final double kCaptureSeconds = 2.0;

  private final Vision m_vision;
  private final Supplier<ChassisSpeeds> m_robotRelativeSpeeds;
  private final Supplier<Angle> m_turretAngle;
  private final Watchdawg m_watchdog = new Watchdawg(VisionCalibration.class);

  private final Map<String, PairStats> m_pairStats = new HashMap<>();

  /**
   * Result timestamp most recently folded into the statistics, per camera. Prevents the same
   * PhotonVision frame from being counted repeatedly across robot loops, which would otherwise make
   * the sample count meaningless and understate the stddev.
   */
  private final Map<String, Double> m_lastCountedTimestamp = new HashMap<>();

  /**
   * Identifies where the robot is physically parked. All captures sharing a station id share one
   * unknown robot pose in the solve, which is what lets several turret angles at the same spot
   * constrain the turret-zero error.
   */
  private int m_stationId = 0;

  /** Identifies one 2-second capture window. Several per station, one per turret angle. */
  private int m_captureId = 0;

  private boolean m_captureActive = false;

  private final String m_turretCameraName;
  private final Transform3d m_robotToTurretPivot;
  private final Transform3d m_turretToCamera;

  /**
   * @param turretCameraName name of the turret-mounted camera, or null if there is not one
   * @param robotToTurretPivot robot origin to the turret rotation axis
   * @param turretToCamera turret frame (at zero) to the camera
   */
  public VisionCalibration(
      Vision vision,
      Supplier<ChassisSpeeds> robotRelativeSpeeds,
      Supplier<Angle> turretAngle,
      String turretCameraName,
      Transform3d robotToTurretPivot,
      Transform3d turretToCamera) {
    m_vision = vision;
    m_robotRelativeSpeeds = robotRelativeSpeeds;
    m_turretAngle = turretAngle;
    m_turretCameraName = turretCameraName;
    m_robotToTurretPivot = robotToTurretPivot;
    m_turretToCamera = turretToCamera;

    SmartDashboard.putData("TuningModes/VisionCal/Capture", captureCommand());
    SmartDashboard.putData("TuningModes/VisionCal/NextStation", nextStationCommand());
    SmartDashboard.putData("TuningModes/VisionCal/ResetStations", resetStationsCommand());
    SmartDashboard.putData("TuningModes/VisionCal/ResetStatistics", resetStatisticsCommand());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    boolean stationary = isStationary();
    Logger.recordOutput("VisionCal/stationary", stationary);
    Logger.recordOutput("VisionCal/stationId", m_stationId);
    Logger.recordOutput("VisionCal/captureId", m_captureId);
    Logger.recordOutput("VisionCal/captureActive", m_captureActive);
    Logger.recordOutput("VisionCal/turretAngleDeg", m_turretAngle.get().in(Degrees));

    // The turret camera's nominal transform depends on the turret angle, so the solver needs the
    // decomposition (pivot, then rotation, then mount) to separate a turret-zero error from a
    // camera-mount error. Logged here for the same reason as robotToCameraNominal in Camera: so the
    // script never keeps its own drifting copy of these constants.
    if (m_turretCameraName != null) {
      Logger.recordOutput("VisionCal/TurretModel/cameraName", m_turretCameraName);
      Logger.recordOutput("VisionCal/TurretModel/robotToTurretPivot",
          new Pose3d(m_robotToTurretPivot.getTranslation(), m_robotToTurretPivot.getRotation()));
      Logger.recordOutput("VisionCal/TurretModel/turretToCamera",
          new Pose3d(m_turretToCamera.getTranslation(), m_turretToCamera.getRotation()));
    }

    List<CameraSample> samples = collectSamples();
    Logger.recordOutput("VisionCal/camerasWithMultiTag", samples.size());

    for (int i = 0; i < samples.size(); i++) {
      for (int j = i + 1; j < samples.size(); j++) {
        logPair(samples.get(i), samples.get(j), stationary);
      }
    }

    m_watchdog.end("periodic");
  }

  /** Cameras that currently have a usable multi-tag solve, with their implied robot pose. */
  private List<CameraSample> collectSamples() {
    List<CameraSample> samples = new ArrayList<>();
    for (Camera camera : m_vision.getCameras()) {
      Optional<Transform3d> fieldToCamera = camera.getLatestFieldToCamera();
      if (fieldToCamera.isEmpty()) {
        continue;
      }
      // Same composition the pose pipeline uses in Camera#getLatestObservations.
      Transform3d fieldToRobot = fieldToCamera.get().plus(camera.getRobotToCamera().inverse());
      samples.add(
          new CameraSample(
              camera.getName(),
              camera.getLatestResultTimestamp(),
              new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation())));
    }
    return samples;
  }

  private void logPair(CameraSample a, CameraSample b, boolean stationary) {
    String key = a.name() + "-" + b.name();
    String root = "VisionCal/Pairs/" + key + "/";

    double skew = Math.abs(a.timestamp() - b.timestamp());
    Logger.recordOutput(root + "timeSkewSeconds", skew);
    if (skew > kMaxPairTimeSkewSeconds) {
      Logger.recordOutput(root + "fresh", false);
      return;
    }
    Logger.recordOutput(root + "fresh", true);

    // Disagreement expressed in camera A's implied robot frame: "where B thinks the robot is,
    // relative to where A thinks it is".
    Transform3d d = new Transform3d(a.robotPose(), b.robotPose());

    double dx = d.getMeasureX().in(Inches);
    double dy = d.getMeasureY().in(Inches);
    double dz = d.getMeasureZ().in(Inches);
    double norm = Meters.of(d.getTranslation().getNorm()).in(Inches);
    double roll = Math.toDegrees(d.getRotation().getX());
    double pitch = Math.toDegrees(d.getRotation().getY());
    double yaw = Math.toDegrees(d.getRotation().getZ());

    Logger.recordOutput(root + "dxInches", dx);
    Logger.recordOutput(root + "dyInches", dy);
    Logger.recordOutput(root + "dzInches", dz);
    Logger.recordOutput(root + "dTranslationNormInches", norm);
    Logger.recordOutput(root + "dRollDeg", roll);
    Logger.recordOutput(root + "dPitchDeg", pitch);
    Logger.recordOutput(root + "dYawDeg", yaw);

    // Speed is logged next to the ungated disagreement so a latency problem (disagreement that
    // grows with speed) is distinguishable from a mounting problem (disagreement that does not).
    Logger.recordOutput(root + "chassisSpeedMetersPerSec", translationalSpeed());

    if (!stationary || !isNewFrame(a) || !isNewFrame(b)) {
      return;
    }

    PairStats stats = m_pairStats.computeIfAbsent(key, k -> new PairStats());
    stats.add(dx, dy, dz, roll, pitch, yaw);
    stats.log(root);
  }

  /**
   * True if this camera has produced a new PhotonVision frame since it was last counted. Robot loops
   * run faster than camera frames, so without this the same measurement would be accumulated many
   * times over.
   */
  private boolean isNewFrame(CameraSample sample) {
    Double last = m_lastCountedTimestamp.get(sample.name());
    if (last != null && last == sample.timestamp()) {
      return false;
    }
    m_lastCountedTimestamp.put(sample.name(), sample.timestamp());
    return true;
  }

  private double translationalSpeed() {
    ChassisSpeeds speeds = m_robotRelativeSpeeds.get();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  private boolean isStationary() {
    ChassisSpeeds speeds = m_robotRelativeSpeeds.get();
    return translationalSpeed() < kStationarySpeedMetersPerSec
        && Math.abs(speeds.omegaRadiansPerSecond) < kStationaryOmegaRadPerSec;
  }

  /**
   * Labels a {@value #kCaptureSeconds}-second window of the log as one capture. The raw per-camera
   * data is logged continuously by {@link Camera}, so a capture only needs to mark which samples
   * belong together.
   *
   * <p>Hold the robot still and press once. Press again after each turret move; all captures taken
   * without moving the robot share a station id and therefore a single unknown robot pose in the
   * solve.
   */
  public Command captureCommand() {
    return Commands.startEnd(() -> m_captureActive = true, () -> {
      m_captureActive = false;
      m_captureId++;
    })
        .withTimeout(Seconds.of(kCaptureSeconds))
        .ignoringDisable(true)
        .withName("VisionCalCapture");
  }

  /**
   * Press after physically moving the robot to a new spot. Move to several distances from the wall
   * -- varying range is what lets the solver tell a mounting rotation error (whose translation error
   * grows with range) apart from a mounting translation error (constant with range).
   */
  public Command nextStationCommand() {
    return Commands.runOnce(() -> m_stationId++)
        .ignoringDisable(true)
        .withName("VisionCalNextStation");
  }

  public Command resetStationsCommand() {
    return Commands.runOnce(() -> {
      m_stationId = 0;
      m_captureId = 0;
      m_captureActive = false;
    }).ignoringDisable(true).withName("VisionCalResetStations");
  }

  public Command resetStatisticsCommand() {
    return Commands.runOnce(() -> {
      m_pairStats.clear();
      m_lastCountedTimestamp.clear();
    }).ignoringDisable(true).withName("VisionCalResetStatistics");
  }

  public int getStationId() {
    return m_stationId;
  }

  private record CameraSample(String name, double timestamp, Pose3d robotPose) {}

  /**
   * Running mean and standard deviation per disagreement axis, via Welford's algorithm. The split
   * matters: mean is the systematic offset error you can correct, stddev is noise you cannot.
   */
  private static class PairStats {
    private static final String[] kAxisNames = {
      "dxInches", "dyInches", "dzInches", "dRollDeg", "dPitchDeg", "dYawDeg"
    };

    private final double[] m_mean = new double[kAxisNames.length];
    private final double[] m_m2 = new double[kAxisNames.length];
    private int m_count = 0;

    void add(double... values) {
      m_count++;
      for (int i = 0; i < values.length; i++) {
        double delta = values[i] - m_mean[i];
        m_mean[i] += delta / m_count;
        m_m2[i] += delta * (values[i] - m_mean[i]);
      }
    }

    void log(String root) {
      Logger.recordOutput(root + "sampleCount", m_count);
      for (int i = 0; i < kAxisNames.length; i++) {
        Logger.recordOutput(root + "mean/" + kAxisNames[i], m_mean[i]);
        Logger.recordOutput(root + "stdDev/" + kAxisNames[i], stdDev(i));
      }
      // The headline number: how far apart the two cameras think the robot is, on average.
      Logger.recordOutput(
          root + "mean/dTranslationNormInches",
          Math.sqrt(m_mean[0] * m_mean[0] + m_mean[1] * m_mean[1] + m_mean[2] * m_mean[2]));
    }

    private double stdDev(int i) {
      return m_count < 2 ? 0.0 : Math.sqrt(m_m2[i] / (m_count - 1));
    }
  }
}
