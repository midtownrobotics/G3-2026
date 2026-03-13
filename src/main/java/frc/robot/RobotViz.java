package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.Watchdawg;

public class RobotViz {
  public final RobotState m_state;
  private final Watchdawg m_watchdog;

  public RobotViz(RobotState state) {
    m_state = state;
    m_watchdog = new Watchdawg(getClass());
  }

  public void periodic() {
    m_watchdog.start();
    Logger.recordOutput(
        "RobotViz/ZeroedPoses", new Pose3d[] { new Pose3d(), new Pose3d(), new Pose3d() });
    Logger.recordOutput("RobotViz/RobotPose", m_state.getRobotPose());
    var turretBasePose = new Pose3d(
        new Translation3d(-0.1, 0.2, 0.5),
        new Rotation3d(0.0, 0.0, m_state.getTurretAngle().in(Radians)));
    Logger.recordOutput(
        "RobotViz/ComponentPoses",
        new Pose3d[] {
            turretBasePose,
            new Pose3d(
                new Translation3d(0.09, 0, 0.19),
                new Rotation3d(0, m_state.getIntakeAngle().in(Radians), 0)),
            turretBasePose.transformBy(
                new Transform3d(
                    new Translation3d(0.1, 0.0, 0.03),
                    new Rotation3d(0.0, m_state.getHoodAngle().in(Radians), 0)))
        });
    m_watchdog.end("robotVizualization");
  }
}
