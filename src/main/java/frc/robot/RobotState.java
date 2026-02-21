package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Controls;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.TransportRoller;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final IntakePivot m_intakePivot;
  public final IntakeRoller m_intakeRoller;
  public final Turret m_turret;
  public final Feeder m_feeder;
  public final Vision m_vision;
  public final TransportRoller m_transportRoller;
  public final Shooter m_shooter;

  public RobotState(
      Controls controls,
      CommandSwerveDrivetrain drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      TransportRoller transportRoller,
      Shooter shooter,
      Hood hood) {
    m_controls = controls;
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_vision = vision;
    m_transportRoller = transportRoller;
    m_shooter = shooter;

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

  public Angle getIntakeAngle() {
    return m_intakePivot.getAngle();
  }

  public Angle getTurretAngle() {
    return m_turret.getAngle();
  }

  public AngularVelocity getShooterSpeed() {
    return m_shooter.getSpeed();
  }

  public Trigger inAllianceZone() {
    return new Trigger(
        () -> DriverStation.getAlliance()
            .map(FieldConstants::getAllianceZone)
            .map(r -> r.contains(m_drive.getPose().getTranslation()))
            .orElse(false))
        .debounce(0.2);
  }

  public Trigger fuelSensorTripped() {
    return m_feeder.fuelSensorTripped();
  }
}
