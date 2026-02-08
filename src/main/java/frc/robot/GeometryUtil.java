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

  /**
   * Converts field-relative position to radial coordinates relative to the hub.
   *
   * @param fieldPosition Field-relative position (x, y) in meters
   * @return RadialCoordinates containing r and theta
   */
  public static RadialCoordinates toRadialCoordinates(Translation2d fieldPosition) {
    return toRadialCoordinates(fieldPosition, null, null);
  }

  /**
   * Converts field-relative position and velocity to radial coordinates.
   *
   * @param fieldPosition Field-relative position (x, y) in meters
   * @param fieldVelocity Field-relative velocity (x_dot, y_dot) in meters/second
   * @return RadialCoordinates containing r, theta, r_dot, and theta_dot
   */
  public static RadialCoordinates toRadialCoordinates(Translation2d fieldPosition,
                                                      Translation2d fieldVelocity) {
    return toRadialCoordinates(fieldPosition, fieldVelocity, null);
  }

  /**
   * Converts field-relative position, velocity, and acceleration to radial coordinates relative to the hub.
   *
   * Radial coordinate formulas:
   * r = sqrt(x^2 + y^2)
   * theta = atan2(y, x)
   *
   * Radial velocity formulas (when velocity is provided):
   * r_dot = (x * x_dot + y * y_dot) / r
   * theta_dot = (x * y_dot - y * x_dot) / r^2
   *
   * Radial acceleration formulas (when acceleration is provided):
   * r_ddot = (x * x_ddot + y * y_ddot + x_dot^2 + y_dot^2 - r * r_dot^2) / r
   * theta_ddot = (x * y_ddot - y * x_ddot + 2 * (x_dot * y_dot - y_dot * x_dot)) / r^2 - 2 * r_dot * theta_dot / r
   *
   * where (x, y) is position relative to hub, (x_dot, y_dot) is velocity, and (x_ddot, y_ddot) is acceleration
   *
   * @param fieldPosition Field-relative position (x, y) in meters
   * @param fieldVelocity Field-relative velocity (x_dot, y_dot) in meters/second (null if not available)
   * @param fieldAcceleration Field-relative acceleration (x_ddot, y_ddot) in meters/second^2 (null if not available)
   * @return RadialCoordinates containing r, theta, and optionally velocities and accelerations
   */
  public static RadialCoordinates toRadialCoordinates(Translation2d fieldPosition,
                                                      Translation2d fieldVelocity,
                                                      Translation2d fieldAcceleration) {
    Translation2d hubPosition2d = FieldConstants.kHubPosition.toTranslation2d();
    Translation2d relativePosition = fieldPosition.minus(hubPosition2d);

    double x = relativePosition.getX();
    double y = relativePosition.getY();

    double r = relativePosition.getNorm();
    Rotation2d theta = new Rotation2d(x, y);

    // If no velocity provided, return position only
    if (fieldVelocity == null) {
      return new RadialCoordinates(r, theta);
    }

    double xDot = fieldVelocity.getX();
    double yDot = fieldVelocity.getY();

    // Compute radial and tangential velocities
    double rDot = (x * xDot + y * yDot) / r;
    double thetaDot = (x * yDot - y * xDot) / (r * r);

    // If no acceleration provided, return position and velocity
    if (fieldAcceleration == null) {
      return new RadialCoordinates(r, theta, rDot, thetaDot);
    }

    double xDotDot = fieldAcceleration.getX();
    double yDotDot = fieldAcceleration.getY();

    // Compute radial and tangential accelerations
    double rDotDot = (x * xDotDot + y * yDotDot + xDot * xDot + yDot * yDot - r * rDot * rDot) / r;
    double thetaDotDot = (x * yDotDot - y * xDotDot + 2 * (xDot * yDot - yDot * xDot)) / (r * r)
                         - 2 * rDot * thetaDot / r;

    return new RadialCoordinates(r, theta, rDot, thetaDot, rDotDot, thetaDotDot);
  }
}
