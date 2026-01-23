package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.turret.Hood;
import frc.robot.subsystems.turret.Shooter;
import frc.robot.subsystems.turret.Turret;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final IntakePivot m_intakePivot;
  public final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Hood m_hood;
  private final Shooter m_shooter;

  public RobotState(Controls controls, CommandSwerveDrivetrain drive, IntakePivot intakePivot,
      IntakeRoller intakeRoller, Turret turret, Hood hood, Shooter shooter) {
    m_controls = controls;
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_hood = hood;
    m_shooter = shooter;

    m_drive.setDefaultCommand(joyStickDrive());
    bindings();
  }

  public void bindings() {
    m_controls.intake().whileTrue(m_intakePivot.setAngleCommand(IntakeGoal.INTAKING.angle))
        .whileFalse(m_intakePivot.setAngleCommand(IntakeGoal.STOW.angle));
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

  public Angle getIntakeAngle() {
    return m_intakePivot.getAngle();
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Angle getTurretYaw() {
    return m_turret.getYawAngle();
  }

}
