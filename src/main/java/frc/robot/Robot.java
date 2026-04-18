package frc.robot;

import java.net.InetAddress;
import java.net.UnknownHostException;

import frc.lib.LoggedCommandScheduler;
import frc.lib.Watchdawg;
import frc.robot.RobotState.ShooterState;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.FieldConstants;
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
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.HoodIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
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
    private final Shooter m_shooter;
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

    public Robot() {
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
            case 0 ->
                "All changes committed";
            case 1 ->
                "Uncommitted changes";
            default ->
                "Unknown";
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
            m_shooter = new Shooter(new ShooterIOTalonFX());
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
            m_shooter = new Shooter(new ShooterIOSim());
            m_turret = new Turret(new TurretIOSim());
        }

        DynamicCamera turretCamera = new DynamicCamera("Turret", 0.4, () -> true);

        Camera rear = new Camera(
                "Rear",
                new Transform3d(
                        new Translation3d(Inches.of(-11.018), Inches.of(7.388), Inches.of(14.444)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.of(180))));
        Camera rearRight = new Camera(
                "Rear Right",
                new Transform3d(
                        new Translation3d(Inches.of(-8.758), Inches.of(-14.541), Inches.of(8.022)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(-33.26 - 90))),
                () -> (DriverStation.isAutonomous() || !turretCamera.isConnected()));
        Camera rearLeft = new Camera(
                "Rear Left",
                new Transform3d(
                        new Translation3d(Inches.of(-7.692), Inches.of(14.396), Inches.of(14.217)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.of(31.475 + 90))),
                () -> (DriverStation.isAutonomous() || !turretCamera.isConnected()));
        Camera frontLeft = new Camera(
                "Front Left",
                new Transform3d(
                        new Translation3d(Inches.of(-7.076), Inches.of(14.525), Inches.of(10.65)),
                        new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(90 - 37.698))));

        m_vision = new Vision(
                (observation) -> m_drive.addVisionMeasurement(
                        observation.pose(), observation.timestamp(), observation.standardDevs()),
                m_drive::getPose,
                m_drive::resetPose,
                rearRight,
                rearLeft,
                rear,
                frontLeft,
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

        m_autoFactory = new AutoFactory(m_drive::getPose, m_drive::resetPose, m_drive::followPath, true, m_drive);

        m_autoRoutines = new AutoRoutines(m_autoFactory, this, m_robotCommands);
        m_autoChooser = new AutoChooser("Do Nothing");

        generateAutoChooser();

        m_hood.setDefaultCommand(m_hood.setAngleCommand(Degrees.zero()));

        m_turret.setDefaultCommand(m_robotCommands.alignTurret());

        m_drive.setDefaultCommand(m_robotCommands.driveCommand());

        m_trimControls = new TrimXboxControls(1);

        configureTrimControlBindings(m_trimControls);

        m_state.isPreparedToShootTrigger()
                .onTrue(m_robotCommands.feedFuel())
                .onFalse(m_robotCommands.stopFeedingFuel());

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

        SmartDashboard.putData("DriveSysid/Rotation/QuasistaticForward", m_drive.sysIdQuasistaticRotation(Direction.kForward));
        SmartDashboard.putData("DriveSysid/Rotation/QuasistaticReverse", m_drive.sysIdQuasistaticRotation(Direction.kReverse));
        SmartDashboard.putData("DriveSysid/Rotation/DynamicForward", m_drive.sysIdDynamicRotation(Direction.kForward));
        SmartDashboard.putData("DriveSysid/Rotation/DynamicReverse", m_drive.sysIdDynamicRotation(Direction.kReverse));

        SmartDashboard.putData("StartSignalLogger", Commands.runOnce(() -> SignalLogger.start()));
        SmartDashboard.putData("StopSignalLogger", Commands.runOnce(() -> SignalLogger.stop()));

        SmartDashboard.putData("Drive/DriveStraightRobotRelative", m_robotCommands.driveStrightRobotRelative());
    }

    private void generateAutoChooser() {
        m_autoChooser.addRoutine("Madtown Left", m_autoRoutines::MadtownLeft);
        m_autoChooser.addRoutine("Madtown Right", m_autoRoutines::MadtownRight);
        m_autoChooser.addRoutine("Hub Swipe Left", m_autoRoutines::HubSwipeLeft);
        m_autoChooser.addRoutine("Hub Swipe Right", m_autoRoutines::HubSwipeRight);

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
    }

    public void configureBindings() {
        m_controls.idle().onTrue(m_robotCommands.idle());

        m_controls.intake().onTrue(m_robotCommands.fill());

        m_controls.defense().onTrue(m_robotCommands.defense());

        m_controls.shoot().onTrue(m_robotCommands.autoAimAndPrepareShootTeleop());
        m_controls.shoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.snowBlow().onTrue(m_robotCommands.snowBlow());
        m_controls.snowBlow().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.setpointShoot().onTrue(m_robotCommands.setPointShoot());
        m_controls.setpointShoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.feedFuel().onTrue(m_robotCommands.feedFuel()).onFalse(m_robotCommands.stopFeedingFuel());

        m_controls.zeroHood().whileTrue(m_robotCommands.zeroTurretHood());

        m_controls.zeroIntake().whileTrue(m_robotCommands.zeroIntake());

        m_controls.toggleShootOnTheMove()
                .onTrue(m_state.setShootOnTheMoveEnabledCommand(() -> !m_state.isShootOnTheMoveEnabled()));
    }

    public void configureTrimControlBindings(TrimControls controls) {
        controls.increaseFlywheelVelocity().onTrue(m_robotCommands.increaseFlywheelVelocity());
        controls.decreaseFlywheelVelocity().onTrue(m_robotCommands.decreaseFlywheelVelocity());

        controls.increaseHoodAngle().onTrue(m_robotCommands.increaseHoodAngle());
        controls.decreaseHoodAngle().onTrue(m_robotCommands.decreaseHoodAngle());

        controls.increaseVelocityCompensation().onTrue(m_robotCommands.increaseVelocityCompensation());
        controls.decreaseVelocityCompensation().onTrue(m_robotCommands.decreaseVelocityCompensation());
        m_vision = new Vision(
                (observation) -> m_drive.addVisionMeasurement(
                        observation.pose().toPose2d(), observation.timestamp(), observation.standardDevs()),
                m_drive::getPose,
                m_drive::resetPose,
                rearRight,
                rearLeft,
                rear,
                frontLeft,
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

        m_autoFactory = new AutoFactory(m_drive::getPose, m_drive::resetPose, m_drive::followPath, true, m_drive);

        m_autoRoutines = new AutoRoutines(m_autoFactory, this, m_robotCommands);
        m_autoChooser = new AutoChooser("Do Nothing");

        generateAutoChooser();

        m_hood.setDefaultCommand(m_hood.setAngleCommand(Degrees.zero()));

        m_turret.setDefaultCommand(m_robotCommands.alignTurret());

        m_drive.setDefaultCommand(m_robotCommands.driveCommand());

        m_trimControls = new TrimXboxControls(1);

        configureTrimControlBindings(m_trimControls);

        m_state.isPreparedToShootTrigger()
                .onTrue(m_robotCommands.feedFuel())
                .onFalse(m_robotCommands.stopFeedingFuel());

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

        SmartDashboard.putData("QuasistaticForward", m_drive.sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("QuasistaticReverse", m_drive.sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("DynamicForward", m_drive.sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("DynamicReverse", m_drive.sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("StartSignalLogger", Commands.runOnce(() -> SignalLogger.start()));
        SmartDashboard.putData("StopSignalLogger", Commands.runOnce(() -> SignalLogger.stop()));

    }

    private void generateAutoChooser() {
        m_autoChooser.addRoutine("Madtown Left", m_autoRoutines::MadtownLeft);
        m_autoChooser.addRoutine("Madtown Right", m_autoRoutines::MadtownRight);
        m_autoChooser.addRoutine("Hub Swipe Left", m_autoRoutines::HubSwipeLeft);
        m_autoChooser.addRoutine("Hub Swipe Right", m_autoRoutines::HubSwipeRight);
        m_autoChooser.addRoutine("Copy 1002 Left", m_autoRoutines::copy1002left);
        m_autoChooser.addRoutine("Copy 1002 Right", m_autoRoutines::copy1002right);

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
        RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
    }

    public void configureBindings() {
        m_controls.idle().onTrue(m_robotCommands.idle());

        m_controls.intake().onTrue(m_robotCommands.fill());

        m_controls.defense().onTrue(m_robotCommands.defense());

        m_controls.shoot().onTrue(m_robotCommands.autoAimAndPrepareShootTeleop());
        m_controls.shoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.snowBlow().onTrue(m_robotCommands.snowBlow());
        m_controls.snowBlow().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.setpointShoot().onTrue(m_robotCommands.setPointShoot());
        m_controls.setpointShoot().onTrue(m_state.setShooterStateCommand(ShooterState.kRev))
                .onFalse(m_state.setShooterStateCommand(ShooterState.kShoot));

        m_controls.feedFuel().onTrue(m_robotCommands.feedFuel()).onFalse(m_robotCommands.stopFeedingFuel());

        m_controls.zeroHood().whileTrue(m_robotCommands.zeroTurretHood());

        m_controls.zeroIntake().whileTrue(m_robotCommands.zeroIntake());

        m_controls.toggleShootOnTheMove()
                .onTrue(m_state.setShootOnTheMoveEnabledCommand(() -> !m_state.isShootOnTheMoveEnabled()));
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

        Logger.recordOutput("Pigeon2/accelerationX", m_drive.getPigeon2().getAccelerationX().getValue());
        Logger.recordOutput("Pigeon2/accelerationY", m_drive.getPigeon2().getAccelerationY().getValue());
        Logger.recordOutput("Pigeon2/accelerationZ", m_drive.getPigeon2().getAccelerationZ().getValue());

        LoggedCommandScheduler.periodic();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        @Override
        public void robotPeriodic
            
        () {

        Logger.recordOutput("Vision/isSOTMEnabled", m_state.isShootOnTheMoveEnabled());

            m_watchdog.start();
            CommandScheduler.getInstance().run();
            m_watchdog.end("commandScheduler");

            m_watchdog.start();
            m_viz.periodic();
            m_watchdog.end("robotVizPeriodic");

            m_state.periodic();

            // Logger.recordOutput("Pigeon2/accelerationX", m_drive.getPigeon2().getAccelerationX().getValue());
            // Logger.recordOutput("Pigeon2/accelerationY", m_drive.getPigeon2().getAccelerationY().getValue());
            // Logger.recordOutput("Pigeon2/accelerationZ", m_drive.getPigeon2().getAccelerationZ().getValue());
            LoggedCommandScheduler.periodic();
        }

        @Override
        public void teleopInit
            
        () {
        if (m_autonomousCommand != null) {
                m_autonomousCommand.cancel();
            }
        }

        @Override
        public void testInit
            
        () {
        CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void driverStationConnected
            
        () {
        CommandScheduler.getInstance()
                    .schedule(m_state.getShootingParameters().setTargetCommand(FieldConstants.getHubPosition2d()));
        }
    }
