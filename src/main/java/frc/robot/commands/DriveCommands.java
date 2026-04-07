package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Watchdawg;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommands {
  private final CommandSwerveDrivetrain m_drive;
  private final Supplier<Double> m_driveLeftSupplier;
  private final Supplier<Double> m_driveForwardSupplier;
  private final Supplier<Double> m_driveRotationSupplier;
  private final RobotState m_state;

  public DriveCommands(CommandSwerveDrivetrain drive,
      Supplier<Double> driveLeftSupplier,
      Supplier<Double> driveForwardSupplier,
      Supplier<Double> driveRotationSupplier,
      RobotState state) {
    m_drive = drive;
    m_state = state;

    SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1.9);
    SlewRateLimiter m_leftLimiter = new SlewRateLimiter(1.9);

    m_driveLeftSupplier = () -> rateLimitInput(driveLeftSupplier.get(), m_leftLimiter);
    m_driveForwardSupplier = () -> rateLimitInput(driveForwardSupplier.get(), m_forwardLimiter);
    // m_driveForwardSupplier = driveForwardSupplier;
    // m_driveLeftSupplier = driveLeftSupplier;
    m_driveRotationSupplier = driveRotationSupplier;

  }

  private double rateLimitInput(double input, SlewRateLimiter limiter) {
    double magnitude = limiter.calculate(input);

    if (input == 0.0) {
      return 0.0;
    }

    if (isScoring()) {
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

          final var speeds = new ChassisSpeeds(
              m_driveForwardSupplier.get()
                  * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                  * Constants.kLinearSpeedMultiplier,
              m_driveLeftSupplier.get()
                  * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                  * Constants.kLinearSpeedMultiplier,
              0);

          double fieldRelativeAngle = m_drive.getPose().getRotation().getRadians();

          speeds.omegaRadiansPerSecond = headingController.calculate(
              fieldRelativeAngle, rotation.get().getMeasure().in(Radians));

          m_drive.setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
          watchdog.end("rotateRobot");
        }, m_drive);
  }

  protected Command rotateRobotForAutonomous(Supplier<Rotation2d> rotation) {
    return rotateRobot(rotation);
  }

  private boolean isScoring() {
    return m_state.inAllianceZone() && m_state.getShooterState() == ShooterState.kShoot;
  }

  private Command joyStickDrive() {
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    return Commands.run(
        () -> {
          watchdog.start();
          double shootingMultiplier = isScoring() ? 0.6 : 1.0;
          double maxSpeed = Constants.kMaxLinearSpeed.in(MetersPerSecond)
              * Constants.kLinearSpeedMultiplier * shootingMultiplier;
          ChassisSpeeds speeds = new ChassisSpeeds(
              m_driveForwardSupplier.get() * maxSpeed,
              m_driveLeftSupplier.get() * maxSpeed,
              Math.copySign(
                  m_driveRotationSupplier.get()
                      * m_driveRotationSupplier.get()
                      * Constants.kAngularMaxSpeed.in(RadiansPerSecond)
                      * Constants.kAngluarSpeedMultiplier,
                  m_driveRotationSupplier.get()));

          m_drive.setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
          watchdog.end("joystickDrive");
        },
        m_drive);
  }

  private Command snakeDrive() {
    final Watchdawg watchdog = new Watchdawg(DriveCommands.class);
    return Commands.run(
        () -> {
          watchdog.start();
          final PIDController headingController = new PIDController(100, 0, 0);
          final boolean snakeDriveActive = !(Math.abs(m_driveRotationSupplier.get()) > 0);

          ChassisSpeeds speeds;
          if (snakeDriveActive) {
            headingController.enableContinuousInput(-Math.PI, Math.PI);

            speeds = new ChassisSpeeds(
                m_driveForwardSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                m_driveLeftSupplier.get()
                    * Constants.kMaxLinearSpeed.in(MetersPerSecond)
                    * Constants.kLinearSpeedMultiplier,
                0);

            Angle headingAngle = Radians.of(
                Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + Math.PI);

            if (Math.abs(speeds.vyMetersPerSecond) > 0.1
                || Math.abs(speeds.vxMetersPerSecond) > 0.1) {
              speeds.omegaRadiansPerSecond = headingController.calculate(
                  m_drive.getPose().getRotation().getRadians(), headingAngle.in(Radians));
            }
          } else {
            speeds = new ChassisSpeeds(
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

          m_drive.setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));

          headingController.close();
          watchdog.end("snakeDrive");
        },
        m_drive);
  }

  protected Command driveCommand() {
    return Constants.kUseWeirdSnakeDrive
        ? snakeDrive()
        : joyStickDrive();
  }
}
