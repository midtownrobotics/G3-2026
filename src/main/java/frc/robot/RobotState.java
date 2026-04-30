package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.GeometryUtil;
import frc.robot.ShootingParameters.ShootingParametersMode;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;

public class RobotState {
  private final Drive m_drive;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Vision m_vision;
  private final Indexer m_indexer;
  private final Flywheel m_shooter;
  private final Hood m_hood;
  private final ShootingParameters m_shootingParameters;

  private final Trigger m_isPreparedToShootTrigger;
	private final Trigger m_isFeedingTrigger;
  private final Trigger m_inAllianceZoneTrigger;

  private final Field2d m_field2d = new Field2d();

  private final LoggedNetworkBoolean m_fixedTurretModeToggle = new LoggedNetworkBoolean("Toggles/FixedTurretMode",
      false);
  private final LoggedNetworkBoolean m_shootOnTheMoveToggle = new LoggedNetworkBoolean("Toggles/ShootOnTheMove", true);
  private final LoggedNetworkBoolean m_autoAimToggle = new LoggedNetworkBoolean("Toggles/AutoAim", false);

  public enum ShooterState {
    kIdle,
    kRev,
    kShoot
  }

  private ShooterState m_shooterState = ShooterState.kIdle;

  private TimeInterpolatableBuffer<Pose2d> m_robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.0);

  public RobotState(
      Drive drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      Indexer indexer,
      Flywheel shooter,
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

		m_inAllianceZoneTrigger = new Trigger(this::inAllianceZone)
      .debounce(0.2, DebounceType.kFalling);

		m_isFeedingTrigger = m_turret.isNearSetpointTrigger()
				.and(() -> m_shooter.isNearSetpoint(RPM.of(500)))
				.and(() -> m_shooter.getSpeed().gt(RPM.of(1600)))
				.and(() -> !getRobotPose().getMeasureY().isNear(FieldConstants.getHubPosition2d().getMeasureY(), Feet.of(2.5)))
				.and(() -> m_shooterState == ShooterState.kShoot)
				.and(() -> m_shootingParameters.getMode() == ShootingParametersMode.kPass)
				.and(RobotModeTriggers.teleop())
				.and(m_inAllianceZoneTrigger.negate());

    m_isPreparedToShootTrigger = m_shooter.isNearSetpointTrigger()
        .and(() -> m_shooterState == ShooterState.kShoot)
        .and(m_hood.isNearSetpointTrigger())
        .and(m_turret.isNearSetpointTrigger())
				.and(() -> m_shootingParameters.getMode() == ShootingParametersMode.kShoot)
        .and(() -> m_shooter.getSetpointSpeed().gt(RPM.of(1200)))
        .debounce(0.1, DebounceType.kFalling);
	
				SmartDashboard.putData("Field", m_field2d);
  }

  public void periodic() {
    m_shootingParameters.periodic();

    double timestamp = Timer.getFPGATimestamp();
    Pose2d robotPose = getRobotPose();


		ChassisSpeeds poseDerivedChassisSpeeds = null;
		if (m_robotPoseBuffer.getInternalBuffer().isEmpty()) {
			poseDerivedChassisSpeeds = getRobotRelativeSpeeds();
		} else {
			var entry = m_robotPoseBuffer.getInternalBuffer().lastEntry();

      Twist2d robotTwist = entry.getValue().log(robotPose);
			double timeslice = Timer.getFPGATimestamp() - entry.getKey();

      poseDerivedChassisSpeeds = new ChassisSpeeds(robotTwist.dx / timeslice, robotTwist.dy / timeslice,
          robotTwist.dtheta / timeslice);
		}

    m_robotPoseBuffer.addSample(timestamp, robotPose);

    Logger.recordOutput("RobotState/fixedTurretModeEnabled", isFixedTurretModeEnabled());
    Logger.recordOutput("RobotState/shootOnTheMoveEnabled", isShootOnTheMoveEnabled());
    Logger.recordOutput("RobotState/inAllianceZoneTrigger", inAllianceZoneTrigger().getAsBoolean());
    Logger.recordOutput("RobotState/isPreparedToShootTrigger", isPreparedToShootTrigger().getAsBoolean());
    Logger.recordOutput("RobotState/shooterMode", getShooterState());
		Logger.recordOutput("RobotState/FieldRelativeRobotSpeeds", getFieldRelativeSpeeds());
		Logger.recordOutput("RobotState/FieldRelativeTurretSpeeds", getFieldRelativeTurretSpeeds());
		Logger.recordOutput("RobotState/PoseDerivedRobotRelativeChassisSpeeds", poseDerivedChassisSpeeds);
		Logger.recordOutput("RobotState/PoseDerivedFieldRelativeChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(poseDerivedChassisSpeeds, robotPose.getRotation()));

    m_field2d.setRobotPose(getRobotPose());
  }

  public ShooterState getShooterState() {
    return m_shooterState;
  }

  public Command setShooterStateCommand(ShooterState state) {
    return Commands.runOnce(() -> m_shooterState = state);
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Pose2d getExpRobotPose(double seconds) {
    return getRobotPose().exp(getRobotRelativeSpeeds().toTwist2d(seconds));
  }

  public Transform2d getRobotToTurretTransform() {
    return new Transform2d(Constants.kRobotToTurret, new Rotation2d(getTurretAngle()));
  }

  public Transform3d getRobotToTurretTransform3d() {
    return new Transform3d(Constants.kRobotToTurret3d, GeometryUtil.rotation3dFromYaw(getTurretAngle()));
  }

  public Transform3d getRobotToTurretCamera() {
    return getRobotToTurretTransform3d().plus(Constants.kTurretToCamera);
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

  public void setFixedTurretMode(boolean enabled) {
    m_fixedTurretModeToggle.set(enabled);
  }

  public ChassisSpeeds getFieldRelativeTurretSpeeds(Pose2d robotPose) {
    ChassisSpeeds robotSpeeds = getFieldRelativeSpeeds();
    double h = Constants.kRobotToTurret.getNorm();
    double theta = robotPose.getRotation().getRadians()
        + Constants.kRobotToTurret.getAngle().getRadians();
    double omega = robotSpeeds.omegaRadiansPerSecond;
    LinearVelocity xDt = MetersPerSecond.of(-h * Math.sin(theta) * omega);
    LinearVelocity yDt = MetersPerSecond.of(h * Math.cos(theta) * omega);
    ChassisSpeeds robotRelativeTurretSpeeds = new ChassisSpeeds(xDt, yDt, RadiansPerSecond.zero());
    return robotSpeeds.plus(robotRelativeTurretSpeeds);
  }

  public Trigger isPreparedToShootTrigger() {
    return m_isPreparedToShootTrigger;
  }

	public Trigger isFeedingTrigger() {
		return m_isFeedingTrigger;
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
    return m_inAllianceZoneTrigger;
  }

  public boolean inAllianceZone() {
    double hubX = FieldConstants.getHubPosition2d().getX();
    double robotX = getRobotPose().getX();

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return robotX < hubX;
    }

    return robotX > hubX;
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
    return m_shootOnTheMoveToggle.get()
        && m_vision.hasRecentAcceptedVision();
    // && !m_drive.hasWheelSlip();
  }

  public ShootingParameters getShootingParameters() {
    return m_shootingParameters;
  }

  public Translation2d calculateFeedTarget() {
    Translation2d hubTranslation = FieldConstants.getHubPosition2d();
    double robotY = getRobotPose().getY();
    double hubY = hubTranslation.getY();

    double targetX = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 1.5
        : FieldConstants.kFieldLength.in(Meters) - 1.5;

    double targetY = hubY - 1.3;

    if (robotY > hubY) {
      targetY = hubY + 1.3;
    }

    return new Translation2d(targetX, targetY);
  }

  public Command setFixedTurretModeEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_fixedTurretModeToggle.set(enabled));
  }

  public Command setShootOnTheMoveEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> m_shootOnTheMoveToggle.set(enabled));
  }

  public Command setShootOnTheMoveEnabledCommand(BooleanSupplier enabled) {
    return Commands.runOnce(() -> m_shootOnTheMoveToggle.set(enabled.getAsBoolean()));
  }
}
