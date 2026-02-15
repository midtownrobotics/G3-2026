package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static final double kSpeedMultiplier = 12;
  public static final LinearVelocity kLinearMaxSpeed = MetersPerSecond.of(7);
  public static final boolean kUseOnTheFlyShooting = false;

  public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d());
}
