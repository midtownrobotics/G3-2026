package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommands {
  public static Command joyStickDrive(CommandSwerveDrivetrain drive, Controls controls) {
    return Commands.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          controls.getDriveForward() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          controls.getDriveLeft() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          Math.copySign(controls.getDriveRotation() * controls.getDriveRotation(), controls.getDriveRotation())
              * Constants.kSpeedMultiplier);

      drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, drive);
  }

  public static Command intakeDrive(CommandSwerveDrivetrain drive, Controls controls) {
    return Commands.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          controls.getDriveForward() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          controls.getDriveLeft() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          0);

      Angle headingAngle = Radians.of(Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

      speeds.omegaRadiansPerSecond = drive.getPose().getRotation().getMeasure().minus(headingAngle).in(Radians) * 0.1;

      drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, drive);
  }
}
