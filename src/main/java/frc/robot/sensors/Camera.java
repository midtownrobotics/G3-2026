package frc.robot.sensors;

import java.sql.ResultSet;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

  public List<PoseObservation> getLatestObservations() {
    List<PoseObservation> observations = new LinkedList<>();

    for (var result : m_camera.getAllUnreadResults()) { 
      DogLog.log("Cameras/" + m_camera.getName() + "/timeStamp", result.getTimestampSeconds());
      if (result.multitagResult.isPresent()) {
        DogLog.log("Cameras/" + m_camera.getName() + "/singleTag", false);
        var multitagResult = result.multitagResult.get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToCamera.getRotation());

        observations.add(
          new PoseObservation(
            result.getTimestampSeconds(), 
            robotPose, 
            multitagResult.fiducialIDsUsed.size()));

       } else if (!result.targets.isEmpty()){
      DogLog.log("Cameras/" + m_camera.getName() + "/singleTag", true);
        var target = result.targets.get(0);

        var tagPose = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltWelded)
                        .getTagPose(target.getFiducialId());
        
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          observations.add(
            new PoseObservation(result.getTimestampSeconds(), robotPose, 1)
          );
        }
       }
      }

      
      DogLog.log("Cameras/" + m_camera.getName() + "/numberOfObservations", observations.size());

      return observations;
    }    

  public boolean hasTargets() {
    PhotonPipelineResult result = m_camera.getLatestResult();

    return result.hasTargets();
  }
}
