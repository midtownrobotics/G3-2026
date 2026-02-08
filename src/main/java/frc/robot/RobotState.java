package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.shooter.Turret;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final IntakePivot m_intakePivot;
  public final Turret m_turret;

  public RobotState(Controls controls, CommandSwerveDrivetrain drive, IntakePivot intakePivot, Turret turret) {
    m_controls = controls;
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_turret = turret;

    m_drive.setDefaultCommand(joyStickDrive());
  }

  public Command joyStickDrive() {
    return Commands.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          m_controls.getDriveForward() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          m_controls.getDriveLeft() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          Math.copySign(m_controls.getDriveRotation() * m_controls.getDriveRotation(), m_controls.getDriveRotation())
              * Constants.kSpeedMultiplier);

      m_drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, m_drive);
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Pose2d getTurretPose() {
    return getRobotPose().transformBy(Constants.kRobotToTurret);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_drive.getChassisSpeeds();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getRobotPose().getRotation());
  }

  public ChassisSpeeds getFieldRelativeTurretSpeeds() {
    ChassisSpeeds robotSpeeds = getFieldRelativeSpeeds();
    double h = Constants.kRobotToTurret.getTranslation().getNorm();
    double theta = getRobotPose().getRotation().getRadians();
    double omega = getFieldRelativeSpeeds().omegaRadiansPerSecond;
    LinearVelocity xDt = MetersPerSecond.of(-h * Math.sin(theta) * omega);
    LinearVelocity yDt = MetersPerSecond.of(h * Math.cos(theta) * omega);
    ChassisSpeeds robotRelativeTurretSpeeds = new ChassisSpeeds(xDt, yDt, RadiansPerSecond.zero());
    return robotSpeeds.plus(robotRelativeTurretSpeeds);
  }

  public Angle getIntakeAngle() {
    return m_intakePivot.getAngle();
  }

  public Angle getTurretAngle() {
    return m_turret.getAngle();
  }

  public Trigger inAllianceZone() {
    return new Trigger(
        () -> DriverStation.getAlliance()
            .map(FieldConstants::getAllianceZone)
            .map(r -> r.contains(m_drive.getPose().getTranslation()))
            .orElse(false))
        .debounce(0.2);
  }

  /**
   * Computes the field-relative angle the turret should point to hit the hub,
   * with compensation for tangential velocity to account for system latency.
   *
   * This method:
   * 1. Converts turret position and velocity to radial coordinates relative to the hub
   * 2. Compensates for tangential velocity to zero out angular motion during flight
   * 3. Accounts for the time delay between angle command and projectile release
   *
   * @param projectileVelocity The exit velocity of the projectile in meters/second
   * @param latencySeconds The total system latency (vision + processing + mechanical) in seconds
   * @return Field-relative angle to command the turret to point at
   */
  public Rotation2d getCompensatedHubAngle(double projectileVelocity, double latencySeconds) {
    // Get turret position and velocity in field coordinates
    Translation2d turretPosition = getTurretPose().getTranslation();
    ChassisSpeeds turretSpeeds = getFieldRelativeTurretSpeeds();
    Translation2d turretVelocity = new Translation2d(
        turretSpeeds.vxMetersPerSecond,
        turretSpeeds.vyMetersPerSecond
    );

    // Convert to radial coordinates relative to the hub
    RadialCoordinates radialCoords = GeometryUtil.toRadialCoordinates(turretPosition, turretVelocity);

    // Calculate time of flight to the hub
    double timeOfFlight = radialCoords.r / projectileVelocity;

    // Total compensation time = latency + time of flight
    double totalCompensationTime = latencySeconds + timeOfFlight;

    // Angular compensation: offset angle by the amount we'll rotate during the compensation period
    // This zeros out the tangential component of velocity imparted to the projectile
    double angularCompensation = radialCoords.thetaDot * totalCompensationTime;

    // Return the compensated field-relative angle
    return radialCoords.theta.plus(Rotation2d.fromRadians(angularCompensation));
  }

  /**
   * Computes the robot-relative angle the turret should point to hit the hub,
   * with compensation for tangential velocity. This converts the field-relative
   * compensated angle to a robot-relative angle for turret control.
   *
   * @param projectileVelocity The exit velocity of the projectile in meters/second
   * @param latencySeconds The total system latency (vision + processing + mechanical) in seconds
   * @return Robot-relative angle (as WPILib Angle) to command the turret to point at
   */
  public Angle getCompensatedHubAngleRobotRelative(double projectileVelocity, double latencySeconds) {
    Rotation2d fieldRelativeAngle = getCompensatedHubAngle(projectileVelocity, latencySeconds);
    Rotation2d robotHeading = getRobotPose().getRotation();

    // Convert field-relative to robot-relative by subtracting robot heading
    Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotHeading);

    return Radians.of(robotRelativeAngle.getRadians());
  }

  /**
   * Computes the robot-relative angle the turret should point to hit the hub,
   * using default compensation parameters from TurretConstants.
   *
   * @return Robot-relative angle (as WPILib Angle) to command the turret to point at
   */
  public Angle getCompensatedHubAngleRobotRelative() {
    return getCompensatedHubAngleRobotRelative(
        TurretConstants.kDefaultProjectileVelocity,
        TurretConstants.kTrackingLatency
    );
  }
}
