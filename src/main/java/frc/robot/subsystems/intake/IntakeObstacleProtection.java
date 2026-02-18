package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.nio.file.Files;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;

public class IntakeObstacleProtection {
  private static final Time kTimeThreshold = Seconds.of(1.5);
  private static final Distance kMaxScanDistance = Meters.of(3.0);

  private IntakeObstacleProtection() {
  }

  public static Trigger getTrigger(RobotState robotState) throws IOException {
    String json = Files.readString(Filesystem.getDeployDirectory().toPath().resolve("navgrid.json"));
    JsonNode root = new ObjectMapper().readTree(json);
    double nodeSize = root.get("nodeSizeMeters").asDouble();
    JsonNode gridNode = root.get("grid");

    boolean[][] grid = new boolean[gridNode.size()][gridNode.get(0).size()];
    for (int row = 0; row < gridNode.size(); row++) {
      for (int col = 0; col < gridNode.get(0).size(); col++) {
        grid[row][col] = gridNode.get(row).get(col).asBoolean();
      }
    }

    return new Trigger(() -> isObstacleTooClose(robotState, grid, nodeSize));
  }

  private static boolean isObstacleTooClose(RobotState robotState, boolean[][] grid, double nodeSize) {
    ChassisSpeeds speeds = robotState.getRobotSpeed();

    double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (totalSpeed <= 0) {
      return false;
    }

    Rotation2d heading = robotState.getRobotPose().getRotation();
    double directionX = heading.getCos();
    double directionY = heading.getSin();

    Translation2d robotPosition = robotState.getRobotPose().getTranslation();
    double distanceToObstacle = distanceToNearestObstacle(robotPosition, directionX, directionY, grid,
        nodeSize);

    if (distanceToObstacle == -1) {
      return false;
    }

    return distanceToObstacle / totalSpeed < kTimeThreshold.in(Seconds);
  }

  private static double distanceToNearestObstacle(
      Translation2d robotPosition, double velocityDirectionX, double velocityDirectionY, boolean[][] grid,
      double nodeSize) {

    double maxDist = kMaxScanDistance.in(Meters);

    for (double distance = 0; distance <= maxDist; distance += nodeSize) {
      int col = (int) ((robotPosition.getX() + velocityDirectionX * distance) / nodeSize);
      int row = (int) ((robotPosition.getY() + velocityDirectionY * distance) / nodeSize);

      if (row < 0 || row >= grid.length || col < 0 || col >= grid[0].length) {
        return -1;
      }

      if (!grid[row][col]) {
        return distance;
      }
    }

    return -1;
  }
}
