package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class ShootingParameters {
  private static final Time kTimeOfFlightTolerance = Seconds.of(0.1);
  private static final int kMaximumIterations = 100;
  private static final Angle kHoodAngleTolerance = Degrees.of(5);
  private static final Angle kTurretAngleTolerance = Degrees.of(5);
  private static final AngularVelocity kFlywhweelVelocityTolerance = RPM.of(50);

  // Takes in a distance in meters and outputs a time in seconds
  private final InterpolatingDoubleTreeMap m_timeOfFlightMap = new InterpolatingDoubleTreeMap();
  // Takes in a distance in meters and outputs an angle in radians 
  private final InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
  // Takes in a distance in meters and outputs an angular velocity in radians per second
  private final InterpolatingDoubleTreeMap m_flywheelVelocityMap = new InterpolatingDoubleTreeMap();

  private final RobotState m_state;

  private Parameters m_currentCycleParameters;

  private final Supplier<Translation2d> m_target;

  public record Parameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity, boolean noShot) {
    public Parameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
      this(turretAngle, hoodAngle, flywheelVelocity, false);
    }
  }

  public ShootingParameters(RobotState state, Supplier<Translation2d> target) {
    m_target = target;
    m_state = state;
  }

  private Time getTimeOfFlight(Translation2d target, Pose2d pose) {
    final Double distance = pose.getTranslation().getDistance(target);
    return Seconds.of(m_timeOfFlightMap.get(distance));
  }

  private Angle getHoodAngle(Translation2d target, Pose2d pose) {
    final Double distance = pose.getTranslation().getDistance(target);
    return Radians.of(m_hoodAngleMap.get(distance));
  }

  private AngularVelocity getFlyWheelVelocity(Translation2d target, Pose2d pose) {
    final Double distance = pose.getTranslation().getDistance(target);
    return RadiansPerSecond.of(m_flywheelVelocityMap.get(distance));
  }

  private Angle getTurretAngle(Translation2d target, Pose2d pose) {
    return new Pose2d(target, new Rotation2d()).minus(pose).getTranslation().getAngle().minus(pose.getRotation())
        .getMeasure();
  }

  private Optional<Pose2d> getVelocityCompensatedRobotPose(Translation2d target, Time ToF, Time oldToF,
      int iterations) {
    final ChassisSpeeds speeds = m_state.getFieldRelativeTurretSpeeds();
    final Pose2d pose = m_state.getTurretPose();

    final var tranform = new Transform2d(
        new Translation2d(speeds.vxMetersPerSecond * ToF.in(Seconds),
            speeds.vyMetersPerSecond * ToF.in(Seconds)),
        new Rotation2d());

    final Pose2d transformedRobotPose = pose.transformBy(tranform);
    final Time newToF = getTimeOfFlight(target, transformedRobotPose);

    if (iterations > kMaximumIterations) {
      return Optional.empty();
    }

    if (ToF.isNear(oldToF, kTimeOfFlightTolerance)) {
      return Optional.of(transformedRobotPose);
    }

    return getVelocityCompensatedRobotPose(target, newToF, ToF, iterations + 1);
  }

  public boolean shootingParametersAreWithinTolerance(Parameters parameters) {
    if (!parameters.turretAngle.isNear(m_state.getTurretAngle(), kTurretAngleTolerance)) {
      return false;
    }

    if (!parameters.hoodAngle.isNear(m_state.getHoodAngle(), kHoodAngleTolerance)) {
      return false;
    }

    if (!parameters.flywheelVelocity.isNear(m_state.getFlyWheelVelocity(), kFlywhweelVelocityTolerance)) {
      return false;
    }

    return true;
  }

  public void periodic() {
    final Translation2d target = m_target.get();
    final Optional<Pose2d> pose = Constants.kUseOnTheFlyShooting
        ? getVelocityCompensatedRobotPose(target, getTimeOfFlight(target, m_state.getTurretPose()),
            Seconds.of(Double.MAX_VALUE), 0)
        : Optional.of(m_state.getTurretPose());

    if (pose.isEmpty()) {
      final Pose2d uncompensatedPose = m_state.getTurretPose();
      m_currentCycleParameters = new Parameters(getTurretAngle(target, uncompensatedPose),
          getHoodAngle(target, uncompensatedPose),
          getFlyWheelVelocity(target, uncompensatedPose),
          true);
      return;
    }

    if (!shootingParametersAreWithinTolerance(m_currentCycleParameters)) {
      final Pose2d uncompensatedPose = m_state.getTurretPose();
      m_currentCycleParameters = new Parameters(getTurretAngle(target, uncompensatedPose),
          getHoodAngle(target, uncompensatedPose),
          getFlyWheelVelocity(target, uncompensatedPose),
          true);
      return;
    }

    m_currentCycleParameters = new Parameters(getTurretAngle(target, pose.get()),
        getHoodAngle(target, pose.get()),
        getFlyWheelVelocity(target, pose.get()));

  }

  public Parameters getParameters() {
    return m_currentCycleParameters;
  }
}
