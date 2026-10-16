package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Speed limits for all chassis commands in board-calibration mode. */
public final class CalibrationDriveLimits {
  public static final double kMaxLinearMetersPerSecond = 0.15;
  public static final double kMaxAngularRadiansPerSecond = 0.2;

  private CalibrationDriveLimits() {
  }

  public static ChassisSpeeds clamp(ChassisSpeeds speeds) {
    double magnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double scale = magnitude > kMaxLinearMetersPerSecond ? kMaxLinearMetersPerSecond / magnitude : 1.0;
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * scale,
        speeds.vyMetersPerSecond * scale,
        MathUtil.clamp(speeds.omegaRadiansPerSecond,
            -kMaxAngularRadiansPerSecond, kMaxAngularRadiansPerSecond));
  }
}
