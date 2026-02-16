package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static final double kSpeedMultiplier = 12;
  public static final LinearVelocity kLinearMaxSpeed = MetersPerSecond.of(7);
  public static final boolean kEnableObstacleProtection = true;
}
