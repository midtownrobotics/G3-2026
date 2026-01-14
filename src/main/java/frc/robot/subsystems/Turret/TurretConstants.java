package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  // PID config 
  final static double yawP = 0;
  final static double yawI = 0;
  final static double yawD = 0;
  final static double pitchP = 0;
  final static double pitchI = 0;
  final static double pitchD = 0;
  final static double yawTolerance = Math.PI / 10;
  final static double pitchTolerance = Math.PI / 10;
  final static double yawPIDRampRate = 0.25;
  final static double pitchPIDRampRate = 0.25;
  // Motor-Pivot config
  final static double motorCurrentLimit = 30;
  final static Angle yawPivotHardLimit = Degrees.of(720);
  final static Angle pitchPivotHardLimit = Degrees.of(180);
  // Flywheel config
  final static double shooterP = 0;
  final static double shooterI = 0;
  final static double shooterD = 0;
  final static double shooterTolerance = Math.PI / 10;
  final static double shooterPIDRampRate = 0.25;
  final static double shooterFlyWheelDiameter = 0;
  final static double shooterMass = 0.5;
  final static double shooterVelocityLoss = 0.5; //Remove one muzzle velocity found empirically
}
