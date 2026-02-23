package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.AllianceFlipUtil;

public class FieldConstants {
  public static final Distance kFieldLength = Meters.of(16.54);
  public static final Distance kFieldWidth = Meters.of(8.07);
  public static final Pose2d kRedAllianceRightSide = new Pose2d(kFieldLength, kFieldWidth,
      Rotation2d.k180deg);
  public static final Pose3d kRedAllianceRightSide3d = new Pose3d(kRedAllianceRightSide);

  public static final Translation2d kAllianceZoneOffset = new Translation2d(4.03, 8.07);

  public static final Distance kAllianceZoneLength = DriverStation.getAlliance().get() == Alliance.Blue ? Meters.of(4.03) : kFieldLength.minus(Meters.of(4.03));

  private static Translation3d kHubPosition = new Translation3d(4.6256194, 4.0346376, 1.8);

  public static final Rectangle2d kBlueAllianceZone = new Rectangle2d(new Translation2d(0.0, 0.0),
      kAllianceZoneOffset);
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
    return AllianceFlipUtil.apply(kHubPosition.toTranslation2d());
  }
}
