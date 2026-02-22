package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

public class Constants {
  public static final double kLinearSpeedMultiplier = 1;
  public static final double kAngluarSpeedMultiplier = 1;

  /** Calculated based on tuner constants. */
  public static final AngularVelocity kAngularMaxSpeed = RadiansPerSecond.of(
    TunerConstants.kSpeedAt12Volts.div(
      Meters.of(Math.hypot(TunerConstants.kFrontLeftXPos.in(Meters), TunerConstants.kFrontLeftYPos.in(Meters)))
    ).in(MetersPerSecond.per(Meters))
  );

  public static final boolean kUseOnTheFlyShooting = false;

  public enum ControlMode {
    FourWay,
    Conventional
  }

  public static final ControlMode kControlMode = ControlMode.FourWay;

  public static final boolean kUseTrimControls = true;
  public static final boolean kUseWeirdSnakeDrive = false;

  public static final Transform2d kRobotToTurret = new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d());
}
