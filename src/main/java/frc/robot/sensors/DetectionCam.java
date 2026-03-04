package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.Logger;

/**
 * Represents a single camera dedicated soley to object detection
 */
@Logged(strategy = Strategy.OPT_IN)
public class DetectionCam {
  private PhotonCamera m_camera;
  private final Logger m_log;

  private static final Angle kFuelCamFovHorizontal = Degrees.of(70.0);
  private static final Angle kFuelCamFovVertical = Degrees.of(44);
  public static final Distance kFuelDiameter = Meters.of(0.150114);
  public static final double kFocalLengthPixels = 500.0;
  private final Alert m_fuelDetectedOutsideOfBounds = new Alert("Fuel Detected Outside Of Bounds", AlertType.kWarning);

  public record DetectionResult(double[] targetX, double[] targetY) {
  }
  
  /**
   * @param name the name of the camera as set in the coprocessor
   */
  public DetectionCam(String name) {
    m_camera = new PhotonCamera(name);
    m_log = new Logger(getClass());
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

  public List<DetectionResult> getLatestDetectionResults() {
    var results = m_camera.getAllUnreadResults().stream()
        .map((result) -> new DetectionResult(getFuelsX(result), getFuelsY(result))).toList();

    m_log.logDetectionResults("detectionResults", results);
    return results;
  }

  private double[] getFuelsX(PhotonPipelineResult result) {
    return result.getTargets().stream().mapToDouble((target) -> {
      Angle yaw = Degrees.of(target.getYaw());
      double normalizedX = yaw.div(kFuelCamFovHorizontal.div(2.0)).in(Units.Value);
      m_fuelDetectedOutsideOfBounds.set(normalizedX > 1.0 || normalizedX < -1.0);
      return Math.max(-1.0, Math.min(1.0, normalizedX));
    }).toArray();
  }

  private double[] getFuelsY(PhotonPipelineResult result) {
    return result.getTargets().stream().mapToDouble((target) -> {
      Angle yaw = Degrees.of(target.getPitch());
      double normalizedY = yaw.div(kFuelCamFovVertical.div(2.0)).in(Units.Value);
      m_fuelDetectedOutsideOfBounds.set(normalizedY > 1.0 || normalizedY < -1.0);
      return Math.max(-1.0, Math.min(1.0, normalizedY));
    }).toArray();
  }

  public boolean hasTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    return result.hasTargets();
  }
}
