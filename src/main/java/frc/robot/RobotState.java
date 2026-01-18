package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;

  public RobotState(Controls controls, CommandSwerveDrivetrain drive) {
    m_controls = controls;
    m_drive = drive;

    m_drive.setDefaultCommand(joyStickDrive());
  }

  public Command joyStickDrive() {
    return Commands.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          m_controls.getDriveForward() * Constants.LINEAR_MAX_SPEED.in(MetersPerSecond) * Constants.SPEED_MULTIPLIER,
          m_controls.getDriveLeft() * Constants.LINEAR_MAX_SPEED.in(MetersPerSecond) * Constants.SPEED_MULTIPLIER,
          Math.copySign(m_controls.getDriveRotation() * m_controls.getDriveRotation(), m_controls.getDriveRotation())
              * Constants.SPEED_MULTIPLIER);

      m_drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, m_drive);
  }

}
