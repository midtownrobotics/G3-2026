package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GeometryUtil {
  public static Rotation3d rotation3dFromYaw(Angle angle) {
    return new Rotation3d(Degrees.zero(), Degrees.zero(), angle);
  }

  public static Transform3d transform3dFromYaw(Angle angle) {
    return new Transform3d(Translation3d.kZero, rotation3dFromYaw(angle));
  }

  public static Transform3d transform3dFromTranslation(Translation3d translation) {
    return new Transform3d(translation, Rotation3d.kZero);
  }

  public static Transform3d transform3dFromRotation(Rotation3d rotation) {
    return new Transform3d(Translation3d.kZero, rotation);
  }

  public static Transform3d transform3dFromPose(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  public static Transform2d transform2dFromTranslation(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  public static Transform2d transform2dFromRotation(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  public static Transform2d transform2dFromPose(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Pose2d pose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  public static Pose2d pose2dFromTransform(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  public static Pose3d pose3dFromTranslation(Translation3d translation) {
    return new Pose3d(translation, Rotation3d.kZero);
  }

  public static Pose3d pose3dFromTransform(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  public static Translation2d flip(Translation2d translation) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FieldConstants.kRedAllianceRightSide.transformBy(transform2dFromTranslation(translation)).getTranslation();
    } else {
      return translation;
    }
  }

  public static Pose2d flip(Pose2d pose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FieldConstants.kRedAllianceRightSide.transformBy(transform2dFromPose(pose));
    } else {
      return pose;
    }
  }

  public static Translation3d flip(Translation3d translation) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FieldConstants.kRedAllianceRightSide3d.transformBy(transform3dFromTranslation(translation))
          .getTranslation();
    } else {
      return translation;
    }
  }

  public static Pose3d flip(Pose3d pose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FieldConstants.kRedAllianceRightSide3d.transformBy(transform3dFromPose(pose));
    } else {
      return pose;
    }
  }
}
