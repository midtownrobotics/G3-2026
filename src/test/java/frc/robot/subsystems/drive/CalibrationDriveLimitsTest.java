package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class CalibrationDriveLimitsTest {
  @Test
  void clampsCombinedTranslationAndRotation() {
    ChassisSpeeds limited = CalibrationDriveLimits.clamp(new ChassisSpeeds(3, 4, -2));
    assertEquals(0.09, limited.vxMetersPerSecond, 1e-12);
    assertEquals(0.12, limited.vyMetersPerSecond, 1e-12);
    assertEquals(-0.2, limited.omegaRadiansPerSecond, 1e-12);
  }

  @Test
  void preservesSpeedsBelowLimits() {
    ChassisSpeeds limited = CalibrationDriveLimits.clamp(new ChassisSpeeds(-0.06, 0.08, 0.1));
    assertEquals(-0.06, limited.vxMetersPerSecond, 1e-12);
    assertEquals(0.08, limited.vyMetersPerSecond, 1e-12);
    assertEquals(0.1, limited.omegaRadiansPerSecond, 1e-12);
  }
}
