package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
  /** Closest the hub-orbit mode will let the commanded radius get, measured from the hub center. */
  private static final Distance kMinOrbitRadius = Meters.of(
      FieldConstants.kHubFaceRadius.in(Meters) + Constants.kRobotLengthWithBumpers.in(Meters) / 2.0);
  private static final Distance kMaxOrbitRadius = Meters.of(8.0);
  private static final double kOrbitLoopPeriodSeconds = 0.02;

  private static final LoggedTunableNumber kOrbitRadiusKp = new LoggedTunableNumber("HubOrbitDrive/RadiusKp", 4.0);

  private final Drive m_drive;
  private final Supplier<Double> m_driveLeftSupplier;
  private final Supplier<Double> m_driveForwardSupplier;
  private final Supplier<Double> m_driveRotationSupplier;
  private final RobotState m_state;

  public DriveCommands(Drive drive,
      Supplier<Double> driveLeftSupplier,
      Supplier<Double> driveForwardSupplier,
      Supplier<Double> driveRotationSupplier,
      RobotState state) {
    m_drive = drive;
    m_state = state;

    SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1.0);
    SlewRateLimiter m_leftLimiter = new SlewRateLimiter(1.0);

    m_driveLeftSupplier = () -> rateLimitInput(driveLeftSupplier.get(), m_leftLimiter);
    m_driveForwardSupplier = () -> rateLimitInput(driveForwardSupplier.get(), m_forwardLimiter);
    m_driveRotationSupplier = driveRotationSupplier;

  }

  /**
   * Gets the robot heading adjusted for alliance perspective.
   * On red alliance, rotates by 180 deg so that "forward" on the joystick
   * always means "away from the driver" regardless of alliance color.
   */
  private Rotation2d getAllianceAdjustedRotation() {
    Rotation2d rotation = m_drive.getPose().getRotation();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return rotation.plus(Rotation2d.kPi);
    }
    return rotation;
  }

  private double rateLimitInput(double input, SlewRateLimiter limiter) {
    double magnitude = limiter.calculate(input);

    if (input == 0.0) {
      return 0.0;
    }

    if (isShooting()) {
      return magnitude;
    }

    return input;
  }

  protected Command rotateRobot(Supplier<Rotation2d> rotation) {
    final PIDController headingController = new PIDController(7, 0, 0);
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    return Commands.run(
        () -> {
          watchdog.start();
          headingController.enableContinuousInput(-Math.PI, Math.PI);

          final var fieldRelativeSpeeds = new ChassisSpeeds(
              m_driveForwardSupplier.get()
                  * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                  * Constants.kLinearSpeedMultiplier,
              m_driveLeftSupplier.get()
                  * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                  * Constants.kLinearSpeedMultiplier,
              0);

          double fieldRelativeAngle = m_drive.getPose().getRotation().getRadians();

          fieldRelativeSpeeds.omegaRadiansPerSecond = headingController.calculate(
              fieldRelativeAngle, rotation.get().getMeasure().in(Radians));

          m_drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAdjustedRotation()));
          watchdog.end("rotateRobot");
        }, m_drive);
  }

  protected Command rotateRobotForAutonomous(Supplier<Rotation2d> rotation) {
    return rotateRobot(rotation);
  }

  private boolean isShooting() {
    return m_state.getShooterState() == ShooterState.kShoot;
  }

  private Command joyStickDrive() {
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    return Commands.run(
        () -> {
          watchdog.start();
          double shootingMultiplier = isShooting() ? 0.25 : 1.0;
          double maxSpeed = Constants.kMaxLinearSpeed.in(MetersPerSecond)
              * Constants.kLinearSpeedMultiplier * shootingMultiplier;
          ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
              m_driveForwardSupplier.get() * maxSpeed,
              m_driveLeftSupplier.get() * maxSpeed,
              Math.copySign(
                  m_driveRotationSupplier.get()
                      * m_driveRotationSupplier.get()
                      * Constants.kAngularMaxSpeed.in(RadiansPerSecond)
                      * Constants.kAngluarSpeedMultiplier
                      * shootingMultiplier,
                  m_driveRotationSupplier.get()));

          Logger.recordOutput("DriveCommands/joystickDriveCommandedChassisSpeeds", ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAdjustedRotation()));
          m_drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAdjustedRotation()));
          watchdog.end("joystickDrive");
        },
        m_drive);
  }

  public Command straightDrive() {
    return Commands.run(() -> m_drive.runVelocity(new ChassisSpeeds(0.25, 0, 0)), m_drive);
  }

  private Command snakeDrive() {
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    return Commands.run(
        () -> {
          watchdog.start();
          final PIDController headingController = new PIDController(100, 0, 0);
          final boolean snakeDriveActive = !(Math.abs(m_driveRotationSupplier.get()) > 0);

          ChassisSpeeds fieldRelativeSpeeds;
          if (snakeDriveActive) {
            headingController.enableContinuousInput(-Math.PI, Math.PI);

            fieldRelativeSpeeds = new ChassisSpeeds(
                m_driveForwardSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                m_driveLeftSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                0);

            Angle headingAngle = Radians.of(
                Math.atan2(fieldRelativeSpeeds.vyMetersPerSecond, fieldRelativeSpeeds.vxMetersPerSecond) + Math.PI);

            if (Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) > 0.1
                || Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) > 0.1) {
              fieldRelativeSpeeds.omegaRadiansPerSecond = headingController.calculate(
                  m_drive.getPose().getRotation().getRadians(), headingAngle.in(Radians));
            }
          } else {
            fieldRelativeSpeeds = new ChassisSpeeds(
                m_driveForwardSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                m_driveLeftSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                Math.copySign(
                    m_driveRotationSupplier.get()
                        * m_driveRotationSupplier.get()
                        * Constants.kAngularMaxSpeed.in(RadiansPerSecond)
                        * Constants.kAngluarSpeedMultiplier,
                    m_driveRotationSupplier.get()));
          }

          m_drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAdjustedRotation()));

          headingController.close();
          watchdog.end("snakeDrive");
        },
        m_drive);
  }

  /**
   * Drive in polar coordinates about the hub instead of field-relative XY.
   *
   * <p>Forward/back on the left stick changes the commanded radius (up moves away from the hub,
   * down moves toward it). Left/right on the left stick arcs around the hub at whatever radius is
   * currently commanded; a PI controller on the radius error holds the robot on that circle while
   * it arcs, so the driver traces the arc rather than a chord. The rotation stick still controls
   * heading normally.
   *
   * <p>"Left" is the driver's left: the tangent is the outward radial rotated -90 deg, which lands
   * on driver-left for both alliances as long as the robot is on our side of the hub.
   */
  protected Command hubOrbitDrive() {
    final PIDController radiusController = new PIDController(kOrbitRadiusKp.get(), 0, 0);
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    // Commanded radius, integrated from the radial stick. Held across loops so tangential
    // motion has a fixed circle to track.
    final double[] targetRadius = new double[1];

    return Commands.run(
        () -> {
          watchdog.start();

          if (Constants.kTuningMode) {
            radiusController.setP(kOrbitRadiusKp.get());
          }

          Translation2d hub = FieldConstants.getHubPosition2d();
          Translation2d hubToRobot = m_drive.getPose().getTranslation().minus(hub);
          double radius = hubToRobot.getNorm();

          // Degenerate: sitting on the hub center, there is no radial direction to hold.
          if (radius < 1e-3) {
            m_drive.runVelocity(new ChassisSpeeds());
            watchdog.end("hubOrbitDrive");
            return;
          }

          double shootingMultiplier = isShooting() ? 0.25 : 1.0;
          double maxSpeed = Constants.kMaxLinearSpeed.in(MetersPerSecond)
              * Constants.kLinearSpeedMultiplier * shootingMultiplier;

          Translation2d outward = hubToRobot.div(radius);
          Translation2d tangentLeft = outward.rotateBy(Rotation2d.kCW_Pi_2);

          double radialInput = m_driveForwardSupplier.get() * maxSpeed;
          targetRadius[0] = MathUtil.clamp(
              targetRadius[0] + radialInput * kOrbitLoopPeriodSeconds,
              kMinOrbitRadius.in(Meters),
              kMaxOrbitRadius.in(Meters));

          // Feedforward the stick so radial motion is not fighting the controller, then correct
          // whatever drift the arcing introduces.
          double radialSpeed = radialInput + radiusController.calculate(radius, targetRadius[0]);
          double tangentialSpeed = m_driveLeftSupplier.get() * maxSpeed;

          Translation2d velocity = outward.times(radialSpeed).plus(tangentLeft.times(tangentialSpeed));

          ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
              velocity.getX(),
              velocity.getY(),
              Math.copySign(
                  m_driveRotationSupplier.get()
                      * m_driveRotationSupplier.get()
                      * Constants.kAngularMaxSpeed.in(RadiansPerSecond)
                      * Constants.kAngluarSpeedMultiplier
                      * shootingMultiplier,
                  m_driveRotationSupplier.get()));

          Logger.recordOutput("DriveCommands/hubOrbit/radius", radius);
          Logger.recordOutput("DriveCommands/hubOrbit/targetRadius", targetRadius[0]);
          Logger.recordOutput("DriveCommands/hubOrbit/radiusError", targetRadius[0] - radius);
          Logger.recordOutput("DriveCommands/hubOrbit/commandedChassisSpeeds", fieldRelativeSpeeds);

          // Field relative, not alliance adjusted: the stick axes mean radial/tangential here, and
          // those directions are already derived from the hub.
          m_drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, m_drive.getPose().getRotation()));
          watchdog.end("hubOrbitDrive");
        },
        m_drive)
        .beforeStarting(() -> {
          // Latch the radius we were sitting at when the mode was engaged.
          targetRadius[0] = MathUtil.clamp(
              m_drive.getPose().getTranslation().getDistance(FieldConstants.getHubPosition2d()),
              kMinOrbitRadius.in(Meters),
              kMaxOrbitRadius.in(Meters));
          radiusController.reset();
        })
        .withName("hubOrbitDrive");
  }

  protected Command driveCommand() {
    return Constants.kUseWeirdSnakeDrive
        ? snakeDrive()
        : joyStickDrive();
  }
}
