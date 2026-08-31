package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.GeometryUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.FieldConstants;

public class ShootingParameters {
  private static final Time kTimeOfFlightTolerance = Seconds.of(0.05);
  private static final int kMaximumIterations = 100;
  private static final double kToFTrimStep = 0.05;
  private static final double kFlywheelVelocityTrimStep = 0.05;
  private static final Angle kHoodAngleTrimStep = Degrees.of(0.5);
  private static final double kLatencyCompensationSeconds = 0.03;

  private ShootingParametersMode m_mode = ShootingParametersMode.kShoot;

  public static enum ShootingParametersMode {
    kPass,
    kShoot
  }

  // Takes in a distance in meters and outputs a time in seconds
  private final InterpolatingDoubleTreeMap m_scoringTimeOfFlightMap = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(Feet.of(6.7).in(Meters), 0.9),
      Map.entry(Feet.of(10.4).in(Meters), 1.4),
      Map.entry(Feet.of(15.2).in(Meters), 1.4),
      Map.entry(Feet.of(20).in(Meters), 1.6)
  // Map.entry(Feet.of(5).in(Meters), 0.7),
  // Map.entry(Feet.of(10).in(Meters), 1.0),
  // Map.entry(Feet.of(20).in(Meters), 1.5));
  );

  private final InterpolatingDoubleTreeMap m_feedingTimeOfFlightMap = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(Feet.of(1).in(Meters), 0.3),
      Map.entry(Feet.of(3).in(Meters), 0.3),
      Map.entry(Feet.of(5).in(Meters), 0.3),
      Map.entry(Feet.of(10).in(Meters), 0.3),
      Map.entry(Feet.of(20).in(Meters), 0.3));

  // Takes in a distance in meters and outputs an angle in radians
  private final InterpolatingDoubleTreeMap m_scoringHoodAngleMap = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(Feet.of(4).in(Meters), Degrees.of(0).in(Radians)),
      Map.entry(Feet.of(5).in(Meters), Degrees.of(5-2).in(Radians)),
      Map.entry(Feet.of(6).in(Meters), Degrees.of(6.5-2).in(Radians)),
      Map.entry(Feet.of(7).in(Meters), Degrees.of(9-2).in(Radians)),
      Map.entry(Feet.of(8).in(Meters), Degrees.of(10-2).in(Radians)),
      Map.entry(Feet.of(9).in(Meters), Degrees.of(10-2).in(Radians)),
      Map.entry(Feet.of(10).in(Meters), Degrees.of(11-2).in(Radians)),
      Map.entry(Feet.of(11).in(Meters), Degrees.of(11-2).in(Radians)),
      Map.entry(Feet.of(12).in(Meters), Degrees.of(12-2).in(Radians)),
      Map.entry(Feet.of(13).in(Meters), Degrees.of(12-2).in(Radians)),
      Map.entry(Feet.of(14).in(Meters), Degrees.of(12-2).in(Radians)),
      Map.entry(Feet.of(15).in(Meters), Degrees.of(12-2).in(Radians)),
      Map.entry(Feet.of(16).in(Meters), Degrees.of(12-2).in(Radians))
  );

  private final InterpolatingDoubleTreeMap m_feedingHoodAngleMap = InterpolatingDoubleTreeMap.ofEntries(
     Map.entry(Feet.of(21-6).in(Meters), Degrees.of(28).in(Radians)),
			Map.entry(Feet.of(26-6).in(Meters), Degrees.of(28).in(Radians)),
			Map.entry(Feet.of(31-6).in(Meters), Degrees.of(28).in(Radians)),
			Map.entry(Feet.of(36-6).in(Meters), Degrees.of(28).in(Radians)),
			Map.entry(Feet.of(41-6).in(Meters), Degrees.of(28).in(Radians)),
			Map.entry(Feet.of(48-6).in(Meters), Degrees.of(28).in(Radians)));

  private final InterpolatingDoubleTreeMap m_scoringFlywheelVelocityMap = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(Feet.of(4).in(Meters), RPM.of(1450+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(5).in(Meters), RPM.of(1450+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(6).in(Meters), RPM.of(1500+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(7).in(Meters), RPM.of(1600+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(8).in(Meters), RPM.of(1600+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(9).in(Meters), RPM.of(1700+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(10).in(Meters), RPM.of(1700+50+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(11).in(Meters), RPM.of(1800+50+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(12).in(Meters), RPM.of(1850+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(13).in(Meters), RPM.of(2000+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(14).in(Meters), RPM.of(2050+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(15).in(Meters), RPM.of(2100+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(16).in(Meters), RPM.of(2200+25+50).in(RadiansPerSecond)),
      Map.entry(Feet.of(25).in(Meters), RPM.of(2600+25+50).in(RadiansPerSecond)) //
  );

  private final InterpolatingDoubleTreeMap m_feedingFlywheelVelocityMap = InterpolatingDoubleTreeMap.ofEntries(
			Map.entry(Feet.of(10-6).in(Meters), RPM.of(1100).in(RadiansPerSecond)),
			Map.entry(Feet.of(15-6).in(Meters), RPM.of(1350).in(RadiansPerSecond)),
      Map.entry(Feet.of(21-6).in(Meters), RPM.of(1650).in(RadiansPerSecond)),
			Map.entry(Feet.of(26-6).in(Meters), RPM.of(2000).in(RadiansPerSecond)),
			Map.entry(Feet.of(31-6).in(Meters), RPM.of(2300).in(RadiansPerSecond)),
			Map.entry(Feet.of(36-6).in(Meters), RPM.of(2500).in(RadiansPerSecond)),
			Map.entry(Feet.of(41-6).in(Meters), RPM.of(2900).in(RadiansPerSecond)),
			Map.entry(Feet.of(48-6).in(Meters), RPM.of(3600).in(RadiansPerSecond))
      );

  private double m_flywheelVelocityModifier = 1;
  private Angle m_hoodAngleModifier = Degrees.of(0);
  private double m_ToFModifier = 0.9;
  private Angle m_turretAngleModifier = Degrees.of(0);

  private final RobotState m_state;

  private Parameters m_currentCycleParameters = new Parameters(Degrees.of(0), Degrees.of(0), RPM.of(0));

  private Translation2d m_target = FieldConstants.getHubPosition2d();
  private final Watchdawg m_watchdog = new Watchdawg(getClass());

  public record Parameters(
      Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity, boolean noShot) {
    public Parameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
      this(turretAngle, hoodAngle, flywheelVelocity, false);
    }
  }

  public ShootingParameters(RobotState state) {
    m_state = state;
  }

  private Time getTimeOfFlight(Distance distanceMeters, InterpolatingDoubleTreeMap map) {
    return Seconds.of(map.get(distanceMeters.in(Meters))).times(m_ToFModifier);
  }

  private Angle getHoodAngle(Distance distance, InterpolatingDoubleTreeMap map) {
    return Radians.of(map.get(distance.in(Meters))).plus(m_hoodAngleModifier);
  }

  private AngularVelocity getFlyWheelVelocity(Distance distance, InterpolatingDoubleTreeMap map) {
    return RadiansPerSecond.of(map.get(distance.in(Meters)))
        .times(m_flywheelVelocityModifier);
  }

  private Angle getTurretAngle(Translation2d turret, Translation2d target, Rotation2d robotRotation) {
    return Degrees.of(target.minus(turret)
        .getAngle()
        .minus(robotRotation)
        .getMeasure()
        .plus(m_turretAngleModifier)
        .plus(Rotations.of(1))
        .in(Degrees) % 360);
  }

  private Optional<Translation2d> getVelocityCompensatedTarget(
      Translation2d turret, Translation2d target, ChassisSpeeds turretSpeeds, Time ToF, Time oldToF,
      InterpolatingDoubleTreeMap tofMap, int iterations) {

    if (iterations > kMaximumIterations) {
      return Optional.empty();
    }

    // The ball inherits the turret's velocity at launch. To cancel this drift,
    // aim at a virtual target shifted opposite to the velocity vector.
    final Translation2d adjustedTarget = target.minus(
        new Translation2d(
            turretSpeeds.vxMetersPerSecond * ToF.in(Seconds),
            turretSpeeds.vyMetersPerSecond * ToF.in(Seconds)));

    if (ToF.isNear(oldToF, kTimeOfFlightTolerance)) {
      // Converged — return the adjusted target using the converged ToF
      return Optional.of(adjustedTarget);
    }

    // Refine ToF based on the distance to the adjusted target
    final Distance distanceToTarget = Meters.of(turret.getDistance(adjustedTarget));
    final Time newToF = getTimeOfFlight(distanceToTarget, tofMap);

    return getVelocityCompensatedTarget(turret, target, turretSpeeds, newToF, ToF, tofMap, iterations + 1);
  }

  public void periodic() {
    m_watchdog.start();

    final InterpolatingDoubleTreeMap tofMap = m_mode == ShootingParametersMode.kPass ? m_feedingTimeOfFlightMap
        : m_scoringTimeOfFlightMap;
    final InterpolatingDoubleTreeMap hoodAngleMap = m_mode == ShootingParametersMode.kPass ? m_feedingHoodAngleMap
        : m_scoringHoodAngleMap;
    final InterpolatingDoubleTreeMap flywheelVelocityMap = m_mode == ShootingParametersMode.kPass
        ? m_feedingFlywheelVelocityMap
        : m_scoringFlywheelVelocityMap;

    final Pose2d expRobotPose = m_state.getExpRobotPose(kLatencyCompensationSeconds);
    final Pose2d expTurretPose = m_state.getTurretPose(expRobotPose);
    final Translation2d expTurretTranslation = expTurretPose.getTranslation();
    // final ChassisSpeeds expTurretSpeeds = m_state.getFieldRelativeTurretSpeeds(expRobotPose);
    final ChassisSpeeds expTurretSpeeds = m_state.getFieldRelativeSpeeds();

		Logger.recordOutput("RobotState/FieldRelativeTurretSpeedsExp", m_state.getFieldRelativeTurretSpeeds(expRobotPose));

    final Distance rawDistanceToTarget = Meters.of(expTurretTranslation.getDistance(m_target));
    final Time rawTimeOfFlightToTarget = getTimeOfFlight(rawDistanceToTarget, tofMap);

    final Optional<Translation2d> adjustedTarget = m_state.isShootOnTheMoveEnabled()
        ? getVelocityCompensatedTarget(
            expTurretTranslation,
            m_target,
            expTurretSpeeds,
            rawTimeOfFlightToTarget,
            Seconds.of(Double.MAX_VALUE),
            tofMap,
            0)
        : Optional.of(m_target);

    final Translation2d target = adjustedTarget.orElse(m_target);

    final Distance distanceToTarget = Meters.of(expTurretTranslation.getDistance(target));

    m_currentCycleParameters = new Parameters(
        getTurretAngle(expTurretTranslation, target, expRobotPose.getRotation()),
        getHoodAngle(distanceToTarget, hoodAngleMap),
        getFlyWheelVelocity(distanceToTarget, flywheelVelocityMap));

    Pose3d targetPose3d = GeometryUtil.pose3dFromTranslation(new Translation3d(target.getX(), target.getY(), 2.3));

    Logger.recordOutput("ShootingParameters/targetPose", targetPose3d.toPose2d());
    Logger.recordOutput("ShootingParameters/targetPose3d", targetPose3d);
    Logger.recordOutput("ShootingParameters/distanceToAdjustedTarget", distanceToTarget);
    Logger.recordOutput("ShootingParameters/distanceToTarget", rawDistanceToTarget);
    Logger.recordOutput("ShootingParameters/tofMultiplier", m_ToFModifier);
    Logger.recordOutput("ShootingParameters/hoodAngleAdjustment", m_hoodAngleModifier.in(Degrees));
    Logger.recordOutput("ShootingParameters/shooterRPMMultiplier", m_flywheelVelocityModifier);
		Logger.recordOutput("ShootingParameters/turretAngleAdjustment", m_turretAngleModifier.in(Degrees));
    Logger.recordOutput("ShootingParameters/mode", m_mode);

    m_watchdog.end("periodic");
  }

  public Parameters getParameters() {
    return m_currentCycleParameters;
  }

  public Rotation2d getTargetRobotRotation() {
    return m_target
        .minus(m_state.getTurretPose().getTranslation())
        .getAngle()
        .minus(new Rotation2d(m_state.getTurretAngle()));
  }

  public void increaseFlywheelVelocity() {
    m_flywheelVelocityModifier += kFlywheelVelocityTrimStep;
  }

  public void decreaseFlywheelVelocity() {
    m_flywheelVelocityModifier -= kFlywheelVelocityTrimStep;
  }

  public void increaseHoodAngle() {
    m_hoodAngleModifier = m_hoodAngleModifier.plus(kHoodAngleTrimStep);
  }

  public void decreaseHoodAngle() {
    m_hoodAngleModifier = m_hoodAngleModifier.minus(kHoodAngleTrimStep);
  }

  public void increaseVelocityCompensation() {
    m_ToFModifier += kToFTrimStep;
  }

  public void decreaseVelocityCompensation() {
    m_ToFModifier -= kToFTrimStep;
  }

  private static final Angle kTurretAngleTrimStep = Degrees.of(1);

  public void increaseTurretAngle() {
    m_turretAngleModifier = m_turretAngleModifier.plus(kTurretAngleTrimStep);
  }

  public void decreaseTurretAngle() {
    m_turretAngleModifier = m_turretAngleModifier.minus(kTurretAngleTrimStep);
  }

  private void setTarget(Translation2d target) {
    m_target = target;
  }

  private void setTarget(Translation2d target, ShootingParametersMode mode) {
    m_target = target;
    m_mode = mode;
  }

	public ShootingParametersMode getMode() {
		return m_mode;
	}

  public Command setTargetCommand(Translation2d target) {
    return Commands.runOnce(() -> setTarget(target)).ignoringDisable(true);
  }

  public Command setTargetCommand(Supplier<Translation2d> target) {
    return Commands.run(() -> setTarget(target.get())).ignoringDisable(true);
  }

  public Command setTargetCommand(Supplier<Translation2d> target, ShootingParametersMode mode) {
    return Commands.run(() -> setTarget(target.get(), mode));
  }
}
