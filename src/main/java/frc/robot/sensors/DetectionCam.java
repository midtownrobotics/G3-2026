package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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

@Logged(strategy = Strategy.OPT_IN)
public class DetectionCam {
  private PhotonCamera m_camera;

  private static final Angle kFuelCamFovHorizontal = Degrees.of(70.0);
  private static final Angle kFuelCamFovVertical = Degrees.of(44);
  public static final Distance kFuelDiameter = Meters.of(0.150114);
  public static final double kFocalLengthPixels = 500.0;
  private final Alert m_fuelDetectedOutsideOfBounds = new Alert("Fuel Detected Outside Of Bounds", AlertType.kWarning);

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

  public double[] getFuelsX() {
    PhotonPipelineResult result = m_camera.getLatestResult();

    return result.getTargets().stream().mapToDouble((target) -> {
      Angle yaw = Degrees.of(target.getYaw());
      double normalizedX = yaw.div(kFuelCamFovHorizontal.div(2.0)).in(Units.Value);
      m_fuelDetectedOutsideOfBounds.set(normalizedX > 1.0 || normalizedX < -1.0);
      return Math.max(-1.0, Math.min(1.0, normalizedX));
    }).toArray();
  }

  public double[] getFuelsY() {
    PhotonPipelineResult result = m_camera.getLatestResult();

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
