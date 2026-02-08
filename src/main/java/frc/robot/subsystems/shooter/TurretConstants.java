package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
  // PID config 
  final static double kYawP = 10;
  final static double kYawI = 0;
  final static double kYawD = 0;
  final static double kYawTolerance = Math.PI / 10;
  final static double kYawPIDRampRate = 0.25;

  final static double kHoodP = 10;
  final static double kHoodI = 0;
  final static double kHoodD = 0;
  final static double kHoodTolerance = Math.PI / 10;
  final static double kHoodPIDRampRate = 0.25;

  final static double kShooterP = 0;
  final static double kShooterI = 0;
  final static double kShooterD = 0;
  final static double kShooterTolerance = Math.PI / 10;
  final static double kShooterRampRate = 0.25;

  // Motor-Pivot config
  final static Current kMotorCurrentLImit = Amps.of(30);
  final static Angle kYawPivotHardLimit = Degrees.of(720);
  final static Angle kYawPivotHardMin = Degrees.of(720);
  final static Angle kYawPivotSoftMin = Degrees.of(720);
  final static Angle kYawPivotSoftLimit = Degrees.of(720);

  final static Angle kHoodPivotHardLimit = Degrees.of(180);

  final static double kYawGearReduction = 48;
  final static double kHoodGearReduction = 48;
  final static Mass kHoodPivotMass = Pounds.of(5);
  final static Mass kYawPivotMass = Pounds.of(20);
  final static Distance kHoodPivotDiameter = Inches.of(6);
  final static Distance kYawPivotDiameter = Inches.of(12);
  final static AngularVelocity kYawMotorMaxAngularVelocity = RPM.of(20);
  final static AngularVelocity kHoodMotorMaxAngularVelocity = RPM.of(20);

  // Flywheel config
  final static Distance kShooterFlywheelDiameter = Inches.of(4);
  final static Mass kShooterMass = Pounds.of(0.5);
  final static double kShooterVelocityLoss = 0.5; //Remove once muzzle velocity found empirically
  final static double kShooterGearReduction = 0;
  final static AngularVelocity kShooterMaxAngularVelocity = RPM.of(5000);

  final static double kCRTRatio1 = 0;
  final static double kCRTRatio2 = 0;

  public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d());

  // Hub tracking compensation constants
  /** Default projectile exit velocity in meters/second (should be calibrated empirically) */
  final static double kDefaultProjectileVelocity = 10.0;

  /** System latency for hub tracking compensation in seconds (vision + processing + mechanical) */
  final static double kTrackingLatency = 0.05; // 50ms
}
