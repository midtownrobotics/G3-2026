package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.GeometryUtil;
import frc.robot.generated.TunerConstants;

public class Constants {
  /**
   * Set to true to enable tunable PID/FF values via NetworkTables dashboard. Set to false for
   * competition to reduce overhead.
   */
  public static final boolean kTuningMode = true;

  public static final double kLinearSpeedMultiplier = 1;
  public static final double kAngluarSpeedMultiplier = 0.9;

  public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(4.5);

  /** Calculated based on tuner constants. */
  public static final AngularVelocity kAngularMaxSpeed = RadiansPerSecond.of(
      kMaxLinearSpeed
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

  public static final boolean kUseWeirdSnakeDrive = false;

  public static final Angle kFixedTurretRotation = Degrees.of(90);

  //public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d());

  public static final Translation3d kRobotToTurret3d = new Translation3d(Inches.of(-3.75), Inches.of(7.25),
      Inches.of(13.25));
  public static final Translation2d kRobotToTurret = kRobotToTurret3d.toTranslation2d();

  public static final Transform3d kTurretToCamera = new Transform3d(Inches.of(6.359), Inches.of(0), Inches.of(1.792),
      GeometryUtil.rotation3dFromPitch(Degrees.of(-24)));

  public static final Distance kRobotWidthWithBumpers = Inches.of(38.438);
  public static final Distance kRobotLengthWithBumpers = Inches.of(31.256);
}
