package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.nio.file.Files;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class IntakeObstacleProtection {
  private final CommandSwerveDrivetrain m_drive;
  private final IntakePivot m_intakePivot;

  private static final LinearVelocity kSpeedThreshold = MetersPerSecond.of(2.44);
  private static final Distance kCheckingDistance = Meters.of(2.74);

  private boolean[][] m_grid;
  private double m_nodeSize;

  public IntakeObstacleProtection(CommandSwerveDrivetrain drive, IntakePivot intake) {
    m_drive = drive;
    m_intakePivot = intake;

    try {
      String json = Files.readString(Filesystem.getDeployDirectory().toPath().resolve("navgrid.json"));
      JsonNode root = new ObjectMapper().readTree(json);
      m_nodeSize = root.get("nodeSizeMeters").asDouble();
      JsonNode grid = root.get("grid");

      m_grid = new boolean[grid.size()][grid.get(0).size()];
      for (int rows = 0; rows < grid.size(); rows++) {
        for (int columns = 0; columns < grid.get(0).size(); columns++) {
          m_grid[rows][columns] = grid.get(rows).get(columns).asBoolean();
        }
      }
    } catch (IOException e) {
      System.err.println("navgrid didn't read" + e.getMessage());
      m_grid = null;
    }

    Trigger obstacleDetected = new Trigger(this::detectObstacle);
    obstacleDetected.onTrue(m_intakePivot.setAngleCommand(Degrees.of(87)));
  }

  private boolean detectObstacle() {
    if (m_grid == null) {
      return false;
    }

    ChassisSpeeds speeds = m_drive.getChassisSpeeds();
    double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (velocity < kSpeedThreshold.in(MetersPerSecond)) {
      return false;
    }

    Pose2d pose = m_drive.getPose();
    Translation2d ahead = pose.getTranslation()
        .plus(new Translation2d(kCheckingDistance.in(Meters), pose.getRotation()));

    int col = (int) (ahead.getX() / m_nodeSize);
    int row = (int) (ahead.getY() / m_nodeSize);

    if (row < 0 || row >= m_grid.length || col < 0 || col >= m_grid[0].length) {
      return false;
    }

    return m_grid[row][col] == false;
  }
}
