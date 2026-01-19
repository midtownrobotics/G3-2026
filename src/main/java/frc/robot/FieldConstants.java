package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  public static final Distance kFieldLength = Meters.of(16.54);
  public static final Distance kFieldWidth = Meters.of(8.07);
  public static final Pose2d kRedAllianceRightSide = new Pose2d(kFieldLength, kFieldWidth,
      Rotation2d.k180deg);
  public static final Pose3d kRedAllianceRightSide3d = new Pose3d(kRedAllianceRightSide);

  public static final Translation2d kAllianceZoneOffset = new Translation2d(4.03, 8.07);

  public static Translation3d kHubPosition = new Translation3d(4.6256194, 4.0346376, 1.8);

  public static final Rectangle2d kBlueAllianceZone = new Rectangle2d(new Translation2d(0.0, 0.0),
      FieldConstants.kAllianceZoneOffset);
  public static final Rectangle2d kRedAllianceZone = new Rectangle2d(
      FieldConstants.kRedAllianceRightSide.getTranslation(),
      FieldConstants.kRedAllianceRightSide.getTranslation().minus(FieldConstants.kAllianceZoneOffset));

  public static Rectangle2d getAllianceZone(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return FieldConstants.kBlueAllianceZone;
    } else {
      return FieldConstants.kRedAllianceZone;
    }
  }
}
