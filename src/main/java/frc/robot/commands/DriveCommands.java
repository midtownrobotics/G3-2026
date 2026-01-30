package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
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

  public static Command snakeDrive(CommandSwerveDrivetrain drive, Controls controls) {
    return Commands.run(() -> {
      PIDController controller = new PIDController(100, 0, 0);

      controller.enableContinuousInput(-Math.PI, Math.PI);

      ChassisSpeeds speeds = new ChassisSpeeds(
          controls.getDriveForward() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          controls.getDriveLeft() * Constants.kLinearMaxSpeed.in(MetersPerSecond) * Constants.kSpeedMultiplier,
          0);

      Angle headingAngle = Radians.of(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + Math.PI);

      DogLog.log("DriveCommands/headingAngle", headingAngle);
      DogLog.log("DriveCommands/currentAngle", drive.getPose().getRotation().getMeasure());
      DogLog.log("DriveCommands/rotationRate",
          controller.calculate(drive.getPose().getRotation().getRadians(), headingAngle.in(Radians)));

      if (speeds.vyMetersPerSecond > 0.1 || speeds.vxMetersPerSecond > 0.1 || speeds.vyMetersPerSecond < -0.1
          || speeds.vxMetersPerSecond < -0.1) {
        speeds.omegaRadiansPerSecond = controller
            .calculate(drive.getPose().getRotation().getRadians(), headingAngle.in(Radians));
      }

      drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, drive);
  }
}
