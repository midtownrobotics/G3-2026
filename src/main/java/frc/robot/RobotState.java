package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
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
  public final CommandSwerveDrivetrain m_drive;
  public final IntakePivot m_intakePivot;
  public final IntakeRoller m_intakeRoller;
  public final Turret m_turret;
  public final Feeder m_feeder;
  public final Vision m_vision;
  public final TransportRoller m_transportRoller;
  public final Shooter m_shooter;
  public final Hood m_hood;

  private RobotMode m_mode = RobotMode.kIdle;

  public enum RobotMode {
    kAutoAim,
    kSnowBlow,
    kIdle,
    kIntake,
    kSetpointShoot,
    kUnjam
  }

  public RobotState(
      CommandSwerveDrivetrain drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      TransportRoller transportRoller,
      Shooter shooter,
      Hood hood) {
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_vision = vision;
    m_transportRoller = transportRoller;
    m_shooter = shooter;
    m_hood = hood;
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Pose2d getTurretPose() {
    return getRobotPose().transformBy(Constants.kRobotToTurret);
  }

  public Trigger getModeTrigger(RobotMode mode) {
    return new Trigger(() -> m_mode == mode);
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
    double theta = getRobotPose().getRotation().getRadians()
        + Constants.kRobotToTurret.getTranslation().getAngle().getRadians();
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
    if (Constants.kUseFixedTurretMode) {
      return Constants.kFixedTurretRotation;
    }
    return m_turret.getAngle();
  }

  public Angle getHoodAngle() {
    return m_hood.getAngle();
  }

  public AngularVelocity getFlyWheelVelocity() {
    return m_shooter.getSpeed();
  }

  public Trigger inAllianceZoneTrigger() {
    return new Trigger(
        () -> DriverStation.getAlliance()
            .or(() -> Optional.of(Alliance.Blue))
            .map(FieldConstants::getAllianceZone)
            .map(r -> r.contains(m_drive.getPose().getTranslation()))
            .orElse(false))
        .debounce(0.2);
  }

  public boolean inAllianceZone() {
    return FieldConstants.getAllianceZone(DriverStation.getAlliance().orElseGet(() -> Alliance.Blue))
        .contains(getRobotPose().getTranslation());
  }

  public Trigger fuelSensorTripped() {
    return m_feeder.fuelSensorTripped();
  }

  public Command setRobotModeCommand(RobotMode mode) {
    return Commands.runOnce(() -> m_mode = mode);
  }
}
