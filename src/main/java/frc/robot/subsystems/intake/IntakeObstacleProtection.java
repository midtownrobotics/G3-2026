package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.nio.file.Files;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;

@Logged(strategy = Strategy.OPT_IN)

public class IntakeObstacleProtection {
  private static final Time kTimeThreshold = Seconds.of(1.5);
  private static final Distance kMaxScanDistance = Meters.of(3.0);

  private static final double kMinApproachSpeed = 2;

  private static final double kIntakeOffsetX = 0.59182;
  private static final double kIntakeHalfWidth = 0.397129;

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

    if (totalSpeed <= kMinApproachSpeed) {
      return false;
    }

    if (totalSpeed <= 0) {
      return false;
    }

    Rotation2d heading = robotState.getRobotPose().getRotation();
    double directionX = speeds.vxMetersPerSecond;
    double directionY = speeds.vyMetersPerSecond;

    Vector<N2> intakeDirection = VecBuilder.fill(heading.getCos(), heading.getSin());
    Vector<N2> robotVelocity = VecBuilder.fill(directionX, directionY);

    if (intakeDirection.dot(robotVelocity) <= 0) {
      return false;
    }

    Translation2d intakeForward = new Translation2d(1, 0).rotateBy(heading);
    Translation2d velocity = new Translation2d(directionX, directionY);

    if (velocity.getX() * intakeForward.getX() + velocity.getY() * intakeForward.getY() <= 0) {
      return false;
    }

    Translation2d robotPosition = robotState.getRobotPose().getTranslation();

    Translation2d leftCorner = robotPosition.plus(
        new Translation2d(kIntakeOffsetX, kIntakeHalfWidth).rotateBy(heading));
    Translation2d rightCorner = robotPosition.plus(
        new Translation2d(kIntakeOffsetX, -kIntakeHalfWidth).rotateBy(heading));

    double leftDist = distanceToNearestObstacle(leftCorner, directionX, directionY, grid, nodeSize);
    double rightDist = distanceToNearestObstacle(rightCorner, directionX, directionY, grid, nodeSize);

    double distanceToObstacle;
    if (leftDist == -1 && rightDist == -1) {
      return false;
    } else if (leftDist == -1) {
      distanceToObstacle = rightDist;
    } else if (rightDist == -1) {
      distanceToObstacle = leftDist;
    } else {
      distanceToObstacle = Math.min(leftDist, rightDist);
    }

    return distanceToObstacle / totalSpeed < kTimeThreshold.in(Seconds);
  }

  private static double distanceToNearestObstacle(
      Translation2d origin, double velocityDirectionX, double velocityDirectionY, boolean[][] grid,
      double nodeSize) {

    double maxDist = kMaxScanDistance.in(Meters);

    for (double distance = 0; distance <= maxDist; distance += nodeSize) {
      int col = (int) ((origin.getX() + velocityDirectionX * distance) / nodeSize);
      int row = (int) ((origin.getY() + velocityDirectionY * distance) / nodeSize);

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
