package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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

  // Takes in a distance in meters and outputs a time in seconds
  private final InterpolatingDoubleTreeMap m_timeOfFlightMap = new InterpolatingDoubleTreeMap();
  // Takes in a distance in meters and outputs an angle in radians 
  private final InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
  // Takes in a distance in meters and outputs an angular velocity in radians per second
  private final InterpolatingDoubleTreeMap m_flywheelVelocityMap = new InterpolatingDoubleTreeMap();

  private final RobotState m_state;

  public record Parameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
  }

  public ShootingParameters(RobotState state) {
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
    return pose.minus(new Pose2d(target, new Rotation2d())).getRotation().getMeasure();
  }

  private Pose2d getVelocityCompensatedRobotPose(Translation2d target, Time ToF, Time oldToF) {
    final ChassisSpeeds speeds = m_state.getFieldRelativeTurretSpeeds();
    final Pose2d pose = m_state.getRobotPose();

    final var tranform = new Transform2d(
        new Translation2d(speeds.vxMetersPerSecond * ToF.in(Seconds),
            speeds.vyMetersPerSecond * ToF.in(Seconds)),
        new Rotation2d());

    final Pose2d transformedRobotPose = pose.transformBy(tranform);
    final Time newToF = getTimeOfFlight(target, transformedRobotPose);

    if (ToF.isNear(oldToF, kTimeOfFlightTolerance)) {
      return transformedRobotPose;
    }

    return getVelocityCompensatedRobotPose(target, newToF, ToF);
  }

  public Parameters getParameters(Translation2d target) {
    var pose = getVelocityCompensatedRobotPose(target, getTimeOfFlight(target, m_state.getRobotPose()),
        Seconds.of(Double.MAX_VALUE));

    return new Parameters(getTurretAngle(target, pose),
        getHoodAngle(target, pose),
        getFlyWheelVelocity(target, pose));
  }
}
