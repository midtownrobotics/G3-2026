package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.nio.file.Files;
import java.util.OptionalDouble;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GeometryUtil;
import frc.robot.RobotState;

@Logged(strategy = Strategy.OPT_IN)
public class IntakeObstacleProtection {
  private static final Time kTimeThreshold = Seconds.of(0.1);
  private static final Distance kMaxScanDistance = Meters.of(0.6);

  private static final double kMinApproachSpeedMPS = 3;

  private static final double kIntakeOffsetXMeters = 0.59182;
  private static final double kIntakeHalfWidthMeters = 0.397129;

  private static boolean[][] cachedGrid = null;
  private static double cachedNodeSize = 0;

  private static Trigger cachedTrigger = null;

  private IntakeObstacleProtection() {
  }

  private static void loadGrid() throws IOException {
    if (cachedGrid != null)
      return;

    String json = Files.readString(Filesystem.getDeployDirectory().toPath().resolve("navgrid.json"));
    JsonNode root = new ObjectMapper().readTree(json);
    cachedNodeSize = root.get("nodeSizeMeters").asDouble();
    JsonNode gridNode = root.get("grid");

    cachedGrid = new boolean[gridNode.size()][gridNode.get(0).size()];
    for (int row = 0; row < gridNode.size(); row++) {
      for (int col = 0; col < gridNode.get(row).size(); col++) {
        cachedGrid[row][col] = gridNode.get(row).get(col).asBoolean();
      }
    }
  }

  public static Trigger getTrigger(RobotState robotState) throws IOException {
    if (cachedTrigger != null)
      return cachedTrigger;
    loadGrid();
    cachedTrigger = new Trigger(() -> isObstacleTooClose(robotState, cachedGrid, cachedNodeSize));
    return cachedTrigger;
  }

  private static boolean isObstacleTooClose(RobotState robotState, boolean[][] grid, double nodeSize) {
    ChassisSpeeds speeds = robotState.getFieldRelativeSpeeds();

    double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    if (totalSpeed <= kMinApproachSpeedMPS) {
      return false;
    }

    Rotation2d heading = robotState.getRobotPose().getRotation();
    double directionX = speeds.vxMetersPerSecond;
    double directionY = speeds.vyMetersPerSecond;

    Vector<N2> intakeDirection = VecBuilder.fill(heading.getCos(), heading.getSin());
    Vector<N2> robotVelocity = VecBuilder.fill(directionX, directionY);
    System.out.println("intake direction obstacl protection");

    DogLog.log("Intakedirectionvector", new double[] { intakeDirection.get(0), intakeDirection.get(1) });
    DogLog.log("Robotdirectionvector", new double[] { robotVelocity.get(0), robotVelocity.get(1) });

    if (intakeDirection.dot(robotVelocity) <= 0) {
      return false;
    }

    Pose2d robotPosition = robotState.getRobotPose();

    Translation2d leftCorner = robotPosition
        .transformBy(
            GeometryUtil.transform2dFromTranslation(new Translation2d(kIntakeOffsetXMeters, kIntakeHalfWidthMeters)))
        .getTranslation();
    Translation2d rightCorner = robotPosition
        .transformBy(
            GeometryUtil.transform2dFromTranslation(new Translation2d(kIntakeOffsetXMeters, -kIntakeHalfWidthMeters)))
        .getTranslation();

    OptionalDouble leftDist = distanceToNearestObstacle(leftCorner, directionX, directionY, grid, nodeSize);
    OptionalDouble rightDist = distanceToNearestObstacle(rightCorner, directionX, directionY, grid, nodeSize);

    DogLog.log("leftintakedistance", leftDist.orElse(-1));
    DogLog.log("rightintakedistance", rightDist.orElse(-1));

    if (leftDist.isEmpty() && rightDist.isEmpty()) {
      return false;
    }

    double distanceToObstacle;
    if (leftDist.isEmpty()) {
      distanceToObstacle = rightDist.getAsDouble();
    } else if (rightDist.isEmpty()) {
      distanceToObstacle = leftDist.getAsDouble();
    } else {
      distanceToObstacle = Math.min(leftDist.getAsDouble(), rightDist.getAsDouble());
    }

    return distanceToObstacle / totalSpeed < kTimeThreshold.in(Seconds);
  }

  private static OptionalDouble distanceToNearestObstacle(
      Translation2d origin, double velocityDirectionX, double velocityDirectionY, boolean[][] grid,
      double nodeSize) {

    double maxDist = kMaxScanDistance.in(Meters);

    for (double distance = 0; distance <= maxDist; distance += nodeSize) {
      int col = (int) ((origin.getX() + velocityDirectionX * distance) / nodeSize);
      int row = (int) ((origin.getY() + velocityDirectionY * distance) / nodeSize);

      if (row < 0 || row >= grid.length || col < 0 || col >= grid[0].length) {
        return OptionalDouble.empty();
      }

      if (grid[row][col]) {
        return OptionalDouble.of(distance);
      }
    }

    return OptionalDouble.empty();
  }
}
