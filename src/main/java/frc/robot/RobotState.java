package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.GeometryUtil;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

public class RobotState {
  private final CommandSwerveDrivetrain m_drive;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Vision m_vision;
  private final Indexer m_indexer;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final ShootingParameters m_shootingParameters;

  private final LoggedNetworkBoolean m_fixedTurretModeToggle = new LoggedNetworkBoolean("Toggles/FixedTurretMode",
      false);
  private final LoggedNetworkBoolean m_shootOnTheMoveToggle = new LoggedNetworkBoolean("Toggles/ShootOnTheMove", true);
  private final LoggedNetworkBoolean m_autoAimToggle = new LoggedNetworkBoolean("Toggles/AutoAim", false);

  public RobotState(
      CommandSwerveDrivetrain drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      Indexer indexer,
      Shooter shooter,
      Hood hood) {
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_vision = vision;
    m_indexer = indexer;
    m_shooter = shooter;
    m_hood = hood;
    m_shootingParameters = new ShootingParameters(this);
  }

  public void periodic() {
    m_shootingParameters.periodic();

    Logger.recordOutput("RobotState/fixedTurretModeEnabled", isFixedTurretModeEnabled());
    Logger.recordOutput("RobotState/shootOnTheMoveEnabled", isShootOnTheMoveEnabled());
    Logger.recordOutput("RobotState/inAllianceZone", inAllianceZone());
    Logger.recordOutput("RobotState/isPreparedToShootTrigger", isPreparedToShootTrigger().getAsBoolean());
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Pose2d getExpRobotPose(double seconds) {
    return getRobotPose().exp(getRobotRelativeSpeeds().toTwist2d(seconds));
  }

  private Transform2d getRobotToTurretTransform() {
    return Constants.kRobotToTurret.plus(GeometryUtil.transform2dFromRotation(new Rotation2d(getTurretAngle())));
  }

  public Pose2d getTurretPose(Pose2d robotPose) {
    return robotPose.plus(getRobotToTurretTransform());
  }

  public Pose2d getTurretPose() {
    return getTurretPose(getRobotPose());
  }

  public Pose2d getExpTurretPose(double seconds) {
    return getTurretPose(getExpRobotPose(seconds));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_drive.getChassisSpeeds();
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotRelativeSpeeds(), getRobotPose().getRotation());
  }

  public ChassisSpeeds getFieldRelativeTurretSpeeds() {
    return getFieldRelativeTurretSpeeds(getRobotPose());
  }

  public ChassisSpeeds getFieldRelativeTurretSpeeds(Pose2d robotPose) {
    ChassisSpeeds robotSpeeds = getFieldRelativeSpeeds();
    double h = Constants.kRobotToTurret.getTranslation().getNorm();
    double theta = robotPose.getRotation().getRadians()
        + Constants.kRobotToTurret.getTranslation().getAngle().getRadians();
    double omega = getFieldRelativeSpeeds().omegaRadiansPerSecond;
    LinearVelocity xDt = MetersPerSecond.of(-h * Math.sin(theta) * omega);
    LinearVelocity yDt = MetersPerSecond.of(h * Math.cos(theta) * omega);
    ChassisSpeeds robotRelativeTurretSpeeds = new ChassisSpeeds(xDt, yDt, RadiansPerSecond.zero());
    return robotSpeeds.plus(robotRelativeTurretSpeeds);
  }

  public Trigger isPreparedToShootTrigger() {
    return m_shooter.isNearSetpointTrigger()//.debounce(0.2, DebounceType.kFalling)
        .and(m_hood.isNearSetpointTrigger())
        .and(m_turret.isNearSetpointTrigger().debounce(0.3, DebounceType.kFalling))
        .and(() -> m_shooter.getSetpointSpeed().gt(RPM.of(500)))
        .debounce(0.1, DebounceType.kFalling);
  }

  public Angle getIntakeAngle() {
    return m_intakePivot.getAngle();
  }

  public Angle getTurretAngle() {
    return m_turret.getAngle();
  }

  public Angle getHoodAngle() {
    return m_hood.getAngle();
  }

  public AngularVelocity getFlyWheelVelocity() {
    return m_shooter.getSpeed();
  }

  public Trigger inAllianceZoneTrigger() {
    return new Trigger(this::inAllianceZone)
        .debounce(0.2);
  }

  public boolean inAllianceZone() {
    return DriverStation.getAlliance()
        .map(FieldConstants::getAllianceZone)
        .map(r -> r.contains(m_drive.getPose().getTranslation()))
        .orElse(false);
  }

  public Trigger fuelSensorTripped() {
    return m_feeder.fuelSensorTripped();
  }

  public boolean isFixedTurretModeEnabled() {
    return m_fixedTurretModeToggle.get();
  }

  public boolean isAutoAimEnabled() {
    return m_autoAimToggle.get();
  }

  public boolean isAutoAimAndFixedTurretModeEnabled() {
    return isAutoAimEnabled() && isFixedTurretModeEnabled();
  }

  public boolean isShootOnTheMoveEnabled() {
    return m_shootOnTheMoveToggle.get();
  }

  public ShootingParameters getShootingParameters() {
    return m_shootingParameters;
  }

  public Translation2d calculateFeedTarget() {
    if (GeometryUtil.flip(getTurretPose()).getMeasureY().lt(FieldConstants.kFieldWidth.div(2))) {
      return GeometryUtil.flip(new Translation2d(FieldConstants.kAllianceZoneOffset.getMeasureX().div(2),
          FieldConstants.kFieldWidth.div(4)));
    }
    return GeometryUtil.flip(new Translation2d(FieldConstants.kAllianceZoneOffset.getMeasureX().div(2),
        FieldConstants.kFieldWidth.div(4).times(3)));
  }

  public Command setFixedTurretModeEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_fixedTurretModeToggle.set(enabled));
  }

  public Command setShootOnTheMoveEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_shootOnTheMoveToggle.set(enabled));
  }
}
