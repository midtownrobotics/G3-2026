package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class RobotViz extends SubsystemBase {
  public RobotState m_state;

  public RobotViz(RobotState state) {
    m_state = state;
  }

  @Override
  public void periodic() {
    DogLog.log("RobotViz/RobotPose", m_state.getRobotPose());
    DogLog.log("RobotViz/ZeroedPoses", new Pose3d[] { new Pose3d(), new Pose3d() });
    DogLog.log("RobotViz/ComponentPoses",
        new Pose3d[] {
            new Pose3d(new Translation3d(-0.1, 0.2, 0.5),
                new Rotation3d(0.0, 0.0, Timer.getFPGATimestamp())),
            new Pose3d(new Translation3d(0.09, 0, 0.19), new Rotation3d(0, m_state.getIntakeAngle().in(Radians), 0)) });
  }
}
