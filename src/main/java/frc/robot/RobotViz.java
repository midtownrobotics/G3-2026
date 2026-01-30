package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class RobotViz {
  public RobotState m_state;

  private Supplier<Angle> turretAngle = () -> {
    Pose2d pose = m_state.getRobotPose().transformBy(new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d()));
    Translation2d hubTranslation = new Translation2d(Meters.of(11.7), Meters.of(4.2));
    return hubTranslation.minus(pose.getTranslation()).getAngle().minus(pose.getRotation()).getMeasure();
  };

  public RobotViz(RobotState state) {
    m_state = state;
  }

  public void updateViz() {

    DogLog.log("RobotViz/RobotPose", m_state.getRobotPose());
    DogLog.log("RobotViz/ZeroedPoses", new Pose3d[] { new Pose3d(), new Pose3d() });
    DogLog.log("RobotViz/ComponentPoses",
        new Pose3d[] {
            new Pose3d(new Translation3d(-0.1, 0.2, 0.5),
                new Rotation3d(0.0, 0.0, m_state.getTurretYaw().in(Radians))),
            new Pose3d(new Translation3d(0.09, 0, 0.19), new Rotation3d(0, m_state.getIntakeAngle().in(Radians), 0)) });
  }
}
