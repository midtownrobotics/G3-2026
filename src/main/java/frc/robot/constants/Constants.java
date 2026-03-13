package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

public class Constants {
  /**
   * Set to true to enable tunable PID/FF values via NetworkTables dashboard. Set to false for
   * competition to reduce overhead.
   */
  public static final boolean kTuningMode = true;

  public static final double kLinearSpeedMultiplier = 1;
  public static final double kAngluarSpeedMultiplier = 0.9;

  /** Calculated based on tuner constants. */
  public static final AngularVelocity kAngularMaxSpeed = RadiansPerSecond.of(
      TunerConstants.kSpeedAt12Volts
          .div(
              Meters.of(
                  Math.hypot(
                      TunerConstants.kFrontLeftXPos.in(Meters),
                      TunerConstants.kFrontLeftYPos.in(Meters))))
          .in(MetersPerSecond.per(Meters)));

  public enum ControlMode {
    FourWay,
    Conventional
  }

  public static final ControlMode kControlMode = ControlMode.FourWay;

  public static final boolean kUseWeirdSnakeDrive = false;

  public static final Angle kFixedTurretRotation = Degrees.of(-90);

  public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d());
}
