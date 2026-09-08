package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.GeometryUtil;

public class FieldConstants {
  public static final Distance kFieldLength = Meters.of(16.54);
  public static final Distance kFieldWidth = Meters.of(8.07);
  public static final Pose2d kRedAllianceRightSide = new Pose2d(kFieldLength, kFieldWidth, Rotation2d.k180deg);
  public static final Pose3d kRedAllianceRightSide3d = new Pose3d(kRedAllianceRightSide);
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final Translation2d kAllianceZoneOffset = new Translation2d(4.03, 8.07);

  private static Translation3d kHubPosition = new Translation3d(4.6256194, 4.0346376, 1.8);

  public static final Rectangle2d kBlueAllianceZone = new Rectangle2d(new Translation2d(0.0, 0.0), kAllianceZoneOffset);
  public static final Rectangle2d kRedAllianceZone = new Rectangle2d(
      kRedAllianceRightSide.getTranslation(),
      kRedAllianceRightSide.getTranslation().minus(kAllianceZoneOffset));

  public static Rectangle2d getAllianceZone(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return kBlueAllianceZone;
    } else {
      return kRedAllianceZone;
    }
  }

  public static Translation2d getHubPosition2d() {
    return GeometryUtil.flip(kHubPosition.toTranslation2d());
  }

  /**
   * Distance from the center of the hub to one of its flat faces (the apothem). Derived from the
   * 2026 Rebuilt welded AprilTag layout: the hub face tags (e.g. 20 and 26) sit 0.6035m from
   * {@link #kHubPosition}.
   */
  public static final Distance kHubFaceRadius = Meters.of(0.6035);

  /**
   * Pose to reset odometry to when the robot is physically staged against the driverstation-facing
   * face of the hub, centered on the field's short axis, with the intake pointed away from the hub
   * (toward our driverstation). Blue-origin; flipped for red.
   */
  public static Pose2d getHubZeroPose() {
    // Intake is the +x side of the robot, so the back bumper is what touches the hub.
    double robotCenterToBackBumper = Constants.kRobotLengthWithBumpers.in(Meters) / 2.0;
    double x = kHubPosition.getX() - kHubFaceRadius.in(Meters) - robotCenterToBackBumper;

    return GeometryUtil.flip(new Pose2d(x, kHubPosition.getY(), Rotation2d.k180deg));
  }
}
