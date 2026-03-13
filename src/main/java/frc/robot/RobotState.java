package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.GeometryUtil;
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

public class RobotState {
  private final CommandSwerveDrivetrain m_drive;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Vision m_vision;
  private final TransportRoller m_transportRoller;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final ShootingParameters m_shootingParameters;

  private RobotMode m_mode = RobotMode.kIdle;
  private boolean m_isFixedTurretModeEnabled = false;
  private boolean m_isShootOnTheMoveEnabled = true;

  private final LoggedNetworkBoolean m_fixedTurretModeToggle = new LoggedNetworkBoolean("Toggles/FixedTurretMode",
      false);
  private final LoggedNetworkBoolean m_shootOnTheMoveToggle = new LoggedNetworkBoolean("Toggles/ShootOnTheMove", true);

  private final Map<RobotMode, Trigger> m_robotModesToTrigger;

  public enum RobotMode {
    kAutoAim,
    kSnowBlow,
    kIdle,
    kIntake,
    kSetpointShoot,
    kUnjam,
    kFullFieldShoot
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
    m_shootingParameters = new ShootingParameters(this);

    m_robotModesToTrigger = Stream.of(RobotMode.values())
        .collect(
            Collectors.toMap(Function.identity(), mode -> new Trigger(() -> m_mode == mode)));

    new Trigger(m_fixedTurretModeToggle)
        .onChange(Commands.defer(() -> setFixedTurretModeEnabledCommand(m_fixedTurretModeToggle.get()), Set.of()));
    new Trigger(m_shootOnTheMoveToggle)
        .onChange(Commands.defer(() -> setShootOnTheMoveEnabledCommand(m_shootOnTheMoveToggle.get()), Set.of()));
  }

  public void periodic() {
    m_shootingParameters.periodic();
    Logger.recordOutput("RobotState/robotMode", getRobotMode());
    Logger.recordOutput("RobotState/fixedTurretModeEnabled", isFixedTurretModeEnabled());
    Logger.recordOutput("RobotState/shootOnTheMoveEnabled", isShootOnTheMoveEnabled());
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

  public Trigger isPreparedToShootTrigger() {
    return m_shooter.isNearSetpointTrigger()
        .and(m_hood.isNearSetpointTrigger())
        .and(m_turret.isNearSetpointTrigger())
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
    return new Trigger(
        () -> DriverStation.getAlliance()
            .or(() -> Optional.of(Alliance.Blue))
            .map(FieldConstants::getAllianceZone)
            .map(r -> r.contains(m_drive.getPose().getTranslation()))
            .orElse(false))
        .debounce(0.2);
  }

  public boolean inAllianceZone() {
    return FieldConstants.getAllianceZone(
        DriverStation.getAlliance().orElseGet(() -> Alliance.Blue))
        .contains(getRobotPose().getTranslation());
  }

  public Trigger fuelSensorTripped() {
    return m_feeder.fuelSensorTripped();
  }

  public RobotMode getRobotMode() {
    return m_mode;
  }

  public boolean isFixedTurretModeEnabled() {
    return m_isFixedTurretModeEnabled;
  }

  public boolean isShootOnTheMoveEnabled() {
    return m_isShootOnTheMoveEnabled;
  }

  public ShootingParameters getShootingParameters() {
    return m_shootingParameters;
  }

  public Command setFixedTurretModeEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_isFixedTurretModeEnabled = enabled);
  }

  public Command setShootOnTheMoveEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_isShootOnTheMoveEnabled = enabled);
  }

  public Command setRobotModeCommand(RobotMode mode) {
    return Commands.runOnce(() -> m_mode = mode);
  }

  public Trigger getModeTrigger(RobotMode mode) {
    return m_robotModesToTrigger.get(mode);
  }
}
