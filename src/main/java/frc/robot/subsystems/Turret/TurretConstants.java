package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class TurretConstants {
  // PID config 
  final static double YAW_P = 0;
  final static double YAW_I = 0;
  final static double YAW_D = 0;
  final static double PITCH_P = 0;
  final static double PITCH_I = 0;
  final static double PITCH_D = 0;
  final static double YAW_TOLERANCE = Math.PI / 10;
  final static double PITCH_TOLERANCE = Math.PI / 10;
  final static double YAW_PID_RAMP_RATE = 0.25;
  final static double PITCH_PID_RAMP_RATE = 0.25;
  final static double SHOOTER_P = 0;
  final static double SHOOTER_I = 0;
  final static double SHOOTER_D = 0;
  final static double SHOOTER_TOLERANCE = Math.PI / 10;
  final static double SHOOTER_RAMP_RATE = 0.25;
  // Motor-Pivot config
  final static Current MOTOR_CURRENT_LIMIT = Amps.of(30);
  final static Angle YAW_PIVOT_HARD_LIMIT = Degrees.of(720);
  final static Angle PITCH_PIVOT_HARD_LIMIT = Degrees.of(180);
  final static double YAW_GEAR_REDUCTION = 0;
  final static double PITCH_GEAR_REDUCTION = 0;
  final static Mass PITCH_PIVOT_MASS = Pounds.of(0);
  final static Mass YAW_PIVOT_MASS = Pounds.of(0);
  final static Distance PITCH_PIVOT_DIAMETER = Inches.of(0);
  final static Distance YAW_PIVOT_DIAMETER = Inches.of(0);
  // Flywheel config
  final static Distance SHOOTER_FLYWHEEL_DIAMETER = Inches.of(4);
  final static Mass SHOOTER_MASS = Pounds.of(0.5);
  final static double SHOOTER_VELOCITY_LOSS = 0.5; //Remove once muzzle velocity found empirically
  final static double SHOOTER_GEAR_REDUCTION = 0;
}
