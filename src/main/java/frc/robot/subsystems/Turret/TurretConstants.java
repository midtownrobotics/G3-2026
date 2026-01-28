package frc.robot.subsystems.Turret;

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

public class TurretConstants {
  // PID config 
  final static double kYawP = 0;
  final static double kYawI = 0;
  final static double kYawD = 0;
  final static double kPitchP = 0;
  final static double kPitchI = 0;
  final static double kPitchD = 0;
  final static double kYawTolerance = Math.PI / 10;
  final static double kPitchTolerance = Math.PI / 10;
  final static double kYawPIDRampRate = 0.25;
  final static double kPitchPIDRampRate = 0.25;
  final static double kShooterP = 0;
  final static double kShooterI = 0;
  final static double kShooterD = 0;
  final static double kShooterTolerance = Math.PI / 10;
  final static double kShooterRampRate = 0.25;
  //CANCoder Chinese Remainder Theorem Gear Ratios
  public static final double kCRTRatio1 = 1.0; // Replace these with actual ratio
  public static final double kCRTRatio2 = 1.0;
  // Motor-Pivot config
  final static Current kMotorCurrentLImit = Amps.of(30);
  final static Angle kYawPivotHardLimit = Degrees.of(360);
  final static Angle kYawPivotHardMin = Degrees.of(-360);
  final static Angle kPitchPivotHardLimit = Degrees.of(59);
  final static Angle kPitchPivotHardMin = Degrees.of(0);
  final static double kYawGearReduction = 0;
  final static double kPitchGearReduction = 0;
  final static Mass kPitchPivotMass = Pounds.of(0);
  final static Mass kYawPivotMass = Pounds.of(0);
  final static Distance kPitchPivotDiameter = Inches.of(0);
  final static Distance kYawPivotDiameter = Inches.of(0);
  final static AngularVelocity kYawMotorMaxAngularVelocity = RPM.of(20);
  final static AngularVelocity kPitchMotorMaxAngularVelocity = RPM.of(20);

  // Flywheel config
  final static Distance kShooterFlywheelDiameter = Inches.of(4);
  final static Mass kShooterMass = Pounds.of(0.5);
  final static double kShooterVelocityLoss = 0.5; //Remove once muzzle velocity found empirically
  final static double kShooterGearReduction = 0;
  final static AngularVelocity kShooterMaxAngularVelocity = RPM.of(5000);
}
