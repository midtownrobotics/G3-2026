package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Set;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.LoggedCommandScheduler;
import frc.lib.Watchdawg;
import frc.robot.RobotState.ShooterState;
import frc.robot.ShootingParameters.ShootingParametersMode;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Ports;
import frc.robot.controls.Controls;
import frc.robot.controls.TrimControls;
import frc.robot.controls.TrimXboxControls;
import frc.robot.controls.XboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Camera;
import frc.robot.sensors.DynamicCamera;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.shooter.TurretIOSim;
import frc.robot.subsystems.shooter.TurretIOTalonFX;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final Controls m_controls;
  private final TrimControls m_trimControls;

  private final Drive m_drive;
  private final Vision m_vision;

  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;

  private final Turret m_turret;
  private final Flywheel m_shooter;
  private final Hood m_hood;

  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

  private final Feeder m_feeder;
  private final Indexer m_indexer;

  private final RobotState m_state;
  private final RobotViz m_viz;

  private final Watchdawg m_watchdog;

  private final RobotCommands m_robotCommands;

  private final LoggedDashboardChooser<Integer> m_cameraPipelineChooser;

  public Robot() {

		RobotController.setBrownoutVoltage(6.5);

    DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());

    // m_pdh.setSwitchableChannel(true);

    // AdvantageKit Logger setup
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    try {
      Logger.recordMetadata(
          "Hostname", InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }

    // Set up data receivers & replay source
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Running a physics simulator, log to NT
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    if (isReal()) {
      m_drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

      m_intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
      m_feeder = new Feeder(new FeederIOTalonFX());
      m_indexer = new Indexer(new IndexerIOTalonFX());
      m_hood = new Hood(new HoodIOTalonFX());
      m_shooter = new Flywheel(new FlywheelIOTalonFX());
      m_turret = new Turret(new TurretIOTalonFX());
    } else {
      m_drive = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

      m_intakePivot = new IntakePivot(new IntakePivotIOSim());
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
      m_feeder = new Feeder(new FeederIOSim());
      m_indexer = new Indexer(new IndexerIOSim());
      m_hood = new Hood(new HoodIOSim());
      m_shooter = new Flywheel(new FlywheelIOSim());
      m_turret = new Turret(new TurretIOSim());
    }

    DynamicCamera turretCamera = new DynamicCamera("Turret", 3, () -> m_turret.getAngle().isNear(Degrees.of(130), Degrees.of(55)));

    Camera rearCamera = new Camera(
        "Rear",
        new Transform3d(
            new Translation3d(Inches.of(-9.394), Inches.of(-12.564), Inches.of(20.659)),
            new Rotation3d(Degrees.zero(), Degrees.of(-14), Degrees.of(160))));
    Camera rightCamera = new Camera(
        "Right",
        new Transform3d(
            new Translation3d(Inches.of(-5.587), Inches.of(-13.648), Inches.of(20.6695)),
            new Rotation3d(Degrees.zero(), Degrees.of(-14), Degrees.of(-60))));
    Camera leftCamera = new Camera(
        "Left",
        new Transform3d(
            new Translation3d(Inches.of(1.054), Inches.of(14.429), Inches.of(10.172)),
            new Rotation3d(Degrees.zero(), Degrees.of(-12), Degrees.of(70))));

    m_vision = new Vision(
        (observation) -> m_drive.addVisionMeasurement(
            observation.pose(), observation.timestamp(), observation.standardDevs()),
        m_drive::getPose,
        m_drive::resetPose,
        rearCamera,
        leftCamera,
        rightCamera,
        turretCamera);

    m_controls = new XboxControls(0);

    m_state = new RobotState(
        m_drive,
        m_intakePivot,
        m_intakeRoller,
        m_turret,
        m_feeder,
        m_vision,
        m_indexer,
        m_shooter,
        m_hood);

    turretCamera.addRobotToCameraSupplier(m_state::getRobotToTurretCamera);

    m_robotCommands = new RobotCommands(
        m_drive,
        m_intakePivot,
        m_intakeRoller,
        m_turret,
        m_feeder,
        m_vision,
        m_indexer,
        m_shooter,
        m_hood,
        m_state,
        m_controls);

    m_viz = new RobotViz(m_state);

    m_cameraPipelineChooser = new LoggedDashboardChooser<Integer>("Vision Pipeline");
    m_cameraPipelineChooser.addDefaultOption("Main Field", 0);
    m_cameraPipelineChooser.addOption("Practice Field", 2);
    m_cameraPipelineChooser.addOption("Johnson", 1);

    m_cameraPipelineChooser.onChange(x -> {
      m_vision.setPipelinesToIndex(x);
    });
		
		SmartDashboard.putData("Vision/setToMainFieldPipeline", Commands.runOnce(() -> {m_vision.setPipelinesToIndex(0); }).ignoringDisable(true).withName("setToMainFieldPipeline"));

    m_autoFactory = new AutoFactory(m_drive::getPose, m_drive::resetPose, m_drive::followPath, true, m_drive);

    m_autoRoutines = new AutoRoutines(m_autoFactory, m_robotCommands, m_drive);
    m_autoChooser = new AutoChooser("Do Nothing");

    generateAutoChooser();

    m_hood.setDefaultCommand(m_hood.setAngleCommand(Degrees.zero()).withName("hoodDefaultStow"));

    m_turret.setDefaultCommand(m_robotCommands.turretTrackShootingParameters());

    m_drive.setDefaultCommand(m_robotCommands.driveCommand());

    m_trimControls = new TrimXboxControls(1);

    configureTrimControlBindings(m_trimControls);

    m_state.isPreparedToShootTrigger().or(m_state.isFeedingTrigger())
        .whileTrue(m_robotCommands.feedFuel());

    m_watchdog = new Watchdawg(getClass());

    configureBindings();

    LoggedCommandScheduler.init(CommandScheduler.getInstance());

    m_state.inAllianceZoneTrigger().and(RobotModeTriggers.disabled().negate())
        .and(RobotModeTriggers.autonomous().negate())
        .whileTrue(m_state.getShootingParameters()
            .setTargetCommand(FieldConstants::getHubPosition2d, ShootingParametersMode.kShoot)
            .withName("setTargetCommandHubPosition"))
        .whileFalse(
            m_state.getShootingParameters().setTargetCommand(m_state::calculateFeedTarget, ShootingParametersMode.kPass)
                .withName("setTargetCommandFeed"));

    m_vision.getHasAcceptedVisionUpdateTrigger().negate().debounce(3.0)
        .onTrue(m_controls.rumbleCommand().withTimeout(Seconds.of(1)));

    m_vision.getHasAcceptedVisionUpdateTrigger().debounce(6.0, DebounceType.kFalling)
        .onTrue(m_controls.pulseRumbleCommand(3, 0.14));

    RobotModeTriggers.teleop().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement());

    SmartDashboard.putData("DriveSysid/Translation/QuasistaticForward", m_drive.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData("DriveSysid/Translation/QuasistaticReverse", m_drive.sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("DriveSysid/Translation/DynamicForward", m_drive.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("DriveSysid/Translation/DynamicReverse", m_drive.sysIdDynamic(Direction.kReverse));

    SmartDashboard.putData("DriveSysid/Rotation/QuasistaticForward",
        m_drive.sysIdQuasistaticRotation(Direction.kForward));
    SmartDashboard.putData("DriveSysid/Rotation/QuasistaticReverse",
        m_drive.sysIdQuasistaticRotation(Direction.kReverse));
    SmartDashboard.putData("DriveSysid/Rotation/DynamicForward", m_drive.sysIdDynamicRotation(Direction.kForward));
    SmartDashboard.putData("DriveSysid/Rotation/DynamicReverse", m_drive.sysIdDynamicRotation(Direction.kReverse));

    SmartDashboard.putData("StartSignalLogger", Commands.runOnce(() -> SignalLogger.start()));
    SmartDashboard.putData("StopSignalLogger", Commands.runOnce(() -> SignalLogger.stop()));

		
		SmartDashboard.putData("Commands/ZeroTurretAngle", m_robotCommands.zeroTurretAngle());
    SmartDashboard.putData("Drive/DriveStraightRobotRelative", m_robotCommands.driveStrightRobotRelative());
  }

  private void generateAutoChooser() {
    m_autoChooser.addRoutine("Madtown Left", m_autoRoutines::MadtownLeft);
    m_autoChooser.addRoutine("Madtown Right", m_autoRoutines::MadtownRight);
    m_autoChooser.addRoutine("Hub Swipe Left", m_autoRoutines::HubSwipeLeft);
    m_autoChooser.addRoutine("Hub Swipe Right", m_autoRoutines::HubSwipeRight);
    m_autoChooser.addRoutine("1002 Left", m_autoRoutines::copy1002left);
    m_autoChooser.addRoutine("1002 Right", m_autoRoutines::copy1002right);
		m_autoChooser.addRoutine("Match 13 Depot", m_autoRoutines::match13Depot);
		m_autoChooser.addRoutine("Right Hub Clean Up", m_autoRoutines::rightHubCleanUp);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
  }

  public void configureBindings() {
    m_controls.idle().onTrue(m_robotCommands.idle());

    m_controls.intake().onTrue(m_robotCommands.fill());
    // m_controls.intake().debounce(0.2, DebounceType.kRising).whileTrue(m_robotCommands.reverseIntake());

    m_controls.unjam().whileTrue(m_robotCommands.reverseFeedFuel());

    m_controls.shoot().onTrue(m_robotCommands.autoAimAndPrepareShootTeleop());
    m_controls.shoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
        .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

    m_controls.snowBlow().onTrue(m_robotCommands.snowBlow());
    m_controls.snowBlow().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
        .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

    m_controls.setpointShoot().onTrue(m_robotCommands.setPointShoot());
    m_controls.setpointShoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
        .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

    m_controls.setpointFeed().onTrue(m_robotCommands.fullFieldFeedShoot());
    m_controls.setpointFeed().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
        .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

    m_controls.feedFuel().onTrue(m_robotCommands.feedFuel()).onFalse(m_robotCommands.stopFeedingFuel());

    m_controls.increaseHoodAngle().onTrue(m_robotCommands.increaseHoodAngle());
    m_controls.decreaseHoodAngle().onTrue(m_robotCommands.decreaseHoodAngle());
    m_controls.increaseTurretAngle().onTrue(m_robotCommands.increaseTurretAngle());
    m_controls.decreaseTurretAngle().onTrue(m_robotCommands.decreaseTurretAngle());

    m_controls.defense().onTrue(m_robotCommands.defense());

    m_controls.zeroHood().whileTrue(m_robotCommands.zeroTurretHood());

    m_controls.zeroIntake().whileTrue(m_robotCommands.zeroIntake());

    m_controls.disableShooting().whileTrue(
        Commands.parallel(
            m_state.setShootOnTheMoveEnabledCommand(() -> false),
            m_robotCommands.shootShooterCommand()))
        .onFalse(m_state.setShootOnTheMoveEnabledCommand(() -> true));

    m_controls.fixedShooter()
        .onTrue(Commands.runOnce(() -> m_state.setFixedTurretMode(!m_state.isFixedTurretModeEnabled())));

    m_controls.toggleShootOnTheMove()
        .onTrue(m_state.setShootOnTheMoveEnabledCommand(() -> !m_state.isShootOnTheMoveEnabled()));


		m_controls.wallAndBulldoze().whileTrue(
			Commands.defer(() -> {
				Distance yOffset = Constants.kRobotWidthWithBumpers.div(2).plus(Feet.of(1));
        Pose2d current = m_drive.getPose();
				Distance nearestY = current.getMeasureY().lt(FieldConstants.getHubPosition2d().getMeasureY()) ? yOffset : FieldConstants.kFieldWidth.minus(yOffset);
				Rotation2d angle = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? Rotation2d.fromDegrees(180): Rotation2d.fromDegrees(0);
				return Commands.parallel(
					Commands.sequence(
            m_autoRoutines.driveToPose(
                new Pose2d(current.getX(), current.getY(), angle)),
								            m_autoRoutines.driveToPose(
                new Pose2d(current.getMeasureX(), nearestY,  angle)),
								  m_autoRoutines.driveToPose(
                new Pose2d(FieldConstants.getHubPosition2d().getMeasureX(), nearestY, angle))
					),
					m_robotCommands.haltTurretAndHoodMovement(),
					m_robotCommands.reverseIntake()
				);
			}, Set.of(m_drive, m_hood, m_turret, m_intakePivot, m_intakeRoller)));
		}

  public void configureTrimControlBindings(TrimControls controls) {
    controls.increaseFlywheelVelocity().onTrue(m_robotCommands.increaseFlywheelVelocity());
    controls.decreaseFlywheelVelocity().onTrue(m_robotCommands.decreaseFlywheelVelocity());

    controls.increaseHoodAngle().onTrue(m_robotCommands.increaseHoodAngle());
    controls.decreaseHoodAngle().onTrue(m_robotCommands.decreaseHoodAngle());

    controls.increaseVelocityCompensation().onTrue(m_robotCommands.increaseVelocityCompensation());
    controls.decreaseVelocityCompensation().onTrue(m_robotCommands.decreaseVelocityCompensation());
  }

  @Override
  public void robotPeriodic() {

    Logger.recordOutput("Vision/isSOTMEnabled", m_state.isShootOnTheMoveEnabled());

    m_watchdog.start();
    CommandScheduler.getInstance().run();
    m_watchdog.end("commandScheduler");

    m_watchdog.start();
    m_viz.periodic();
    m_watchdog.end("robotVizPeriodic");

    m_state.periodic();
		
		Logger.recordOutput("CanBusUsage/Drive", Ports.driveCanBus.getStatus().BusUtilization);
		Logger.recordOutput("CanBusUsage/Mechs", Ports.primaryCanBus.getStatus().BusUtilization);

		Logger.recordOutput("matchTime", DriverStation.getMatchTime());

    // Logger.recordOutput("Pigeon2/accelerationX", m_drive.getPigeon2().getAccelerationX().getValue());
    // Logger.recordOutput("Pigeon2/accelerationY", m_drive.getPigeon2().getAccelerationY().getValue());
    // Logger.recordOutput("Pigeon2/accelerationZ", m_drive.getPigeon2().getAccelerationZ().getValue());

    LoggedCommandScheduler.periodic();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void driverStationConnected() {
    CommandScheduler.getInstance()
        .schedule(m_state.getShootingParameters().setTargetCommand(FieldConstants.getHubPosition2d()));
  }
}
