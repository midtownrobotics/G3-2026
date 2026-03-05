package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.Logger;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ControlMode;
import frc.robot.controls.ConventionalControls;
import frc.robot.controls.ConventionalXboxControls;
import frc.robot.controls.DriveControls;
import frc.robot.controls.FourWayControls;
import frc.robot.controls.FourWayXboxControls;
import frc.robot.controls.TrimControls;
import frc.robot.controls.TrimXboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Camera;
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
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final DriveControls m_controls;
  private final TrimControls m_trimControls;

  private final CommandSwerveDrivetrain m_drive;
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
  private final TransportRoller m_transportRoller;

  private final RobotState m_state;
  private final RobotViz m_viz;

  // private final PowerDistribution m_pdh;

  private final RobotCommands m_robotCommands;

  private final Logger m_log = new Logger(getClass());

  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);
    // m_pdh = new PowerDistribution();
    // m_pdh.setSwitchableChannel(true);

    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DataLogManager.start();
    Epilogue.bind(this);

    m_drive = TunerConstants.createDrivetrain();
    m_intakePivot = new IntakePivot();
    m_intakeRoller = new IntakeRoller();
    m_feeder = new Feeder();
    m_transportRoller = new TransportRoller();
    m_hood = new Hood();
    m_shooter = new Shooter();
    m_turret = new Turret();

    Camera rear = new Camera("Rear",
        new Transform3d(new Translation3d(Inches.of(-11.018), Inches.of(7.388), Inches.of(14.444)),
            new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.of(180))));
    Camera rearRight = new Camera("Rear Right",
        new Transform3d(new Translation3d(Inches.of(-8.758), Inches.of(-14.541), Inches.of(8.022)),
            new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(-33.26 - 90))));
    Camera rearLeft = new Camera("Rear Left",
        new Transform3d(new Translation3d(Inches.of(-7.692), Inches.of(14.396), Inches.of(14.217)),
            new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.of(31.475 + 90))));
    Camera frontLeft = new Camera("Front Left",
        new Transform3d(new Translation3d(Inches.of(-7.076), Inches.of(14.525), Inches.of(10.65)),
            new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(90 - 37.698))));

    m_vision = new Vision(
        (observation) -> m_drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp()),
        m_drive::getPose,
        rearRight,
        rearLeft,
        rear,
        frontLeft);

    m_state = new RobotState(
        m_drive,
        m_intakePivot,
        m_intakeRoller,
        m_turret,
        m_feeder,
        m_vision,
        m_transportRoller,
        m_shooter,
        m_hood);

    if (Constants.kControlMode == ControlMode.Conventional) {
      var controls = new ConventionalXboxControls(0);
      configureConventionalBindings(controls);
      m_controls = controls;
    } else {
      var controls = new FourWayXboxControls(0);
      configureFourWayBindings(controls);
      m_controls = controls;
    }

    m_robotCommands = new RobotCommands(m_drive, m_intakePivot, m_intakeRoller, m_turret, m_feeder, m_vision,
        m_transportRoller, m_shooter, m_hood, m_state, m_controls);

    m_viz = new RobotViz(m_state);

    m_autoFactory = new AutoFactory(
        m_drive::getPose,
        m_drive::resetPose,
        m_drive::followPath,
        true,
        m_drive);

    m_autoRoutines = new AutoRoutines(m_autoFactory, this, m_robotCommands);
    m_autoChooser = new AutoChooser("Do Nothing");

    generateAutoChooser();

    m_hood.setDefaultCommand(m_robotCommands.alignHood());

    m_turret.setDefaultCommand(m_robotCommands.alignTurret());

    m_drive.setDefaultCommand(m_robotCommands.driveCommand());

    m_trimControls = new TrimXboxControls(1);
    configureTrimControlBindings(m_trimControls);

    m_robotCommands.parametersHasShot().onTrue(m_robotCommands.runFeeders())
        .onFalse(m_robotCommands.stopFeeders());

    m_transportRoller.setDefaultCommand(m_transportRoller.setVoltageCommand(Volts.of(-10)));
  }

  private void generateAutoChooser() {
    m_autoChooser.addRoutine("Left Depot Shoot", m_autoRoutines::pickupDepotAndShoot);
    m_autoChooser.addRoutine("Depot -> Left Start", m_autoRoutines::depotToLeftStart);
    m_autoChooser.addRoutine("Depot -> Mid Left", m_autoRoutines::depotToMidLeft);
    m_autoChooser.addRoutine("Left Start -> Center -> Mid-Left", m_autoRoutines::leftStartToCenter);
    m_autoChooser.addRoutine("Left Start -> Depot", m_autoRoutines::leftStartToDepot);
    m_autoChooser.addRoutine("Mid Left -> Depot", m_autoRoutines::midLeftToDepot);
    m_autoChooser.addRoutine("Mid Right -> Outpost", m_autoRoutines::midRightToOutpost);
    m_autoChooser.addRoutine("Mid Start -> Depot", m_autoRoutines::midStartToDepot);
    m_autoChooser.addRoutine("Mid Start -> Left Start", m_autoRoutines::midStartToLeftStart);
    m_autoChooser.addRoutine("Outpost -> Mid Right", m_autoRoutines::outpostToMidRight);
    m_autoChooser.addRoutine("Right Start -> Center -> Mid-Right", m_autoRoutines::rightStartToCenter);
    m_autoChooser.addRoutine("Right Start -> Steal Balls -> Left Start", m_autoRoutines::rightToStealBalls);
    m_autoChooser.addRoutine("Left Start -> Steal Balls -> Right Start", m_autoRoutines::leftToStealBalls);
    m_autoChooser.addRoutine("Left Start -> Right Start", m_autoRoutines::leftStartToRightStart);
    m_autoChooser.addRoutine("Right Start -> Left Start", m_autoRoutines::rightStartToleftStart);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
  }

  public void configureConventionalBindings(ConventionalControls controls) {
    controls.shoot().onTrue(m_robotCommands.shoot()).onFalse(m_robotCommands.stopShooting());
    controls.intake().onTrue(m_robotCommands.runIntake()).onFalse(m_robotCommands.stowIntake());
  }

  public void configureFourWayBindings(FourWayControls controls) {
    controls.idle().onTrue(m_robotCommands.idle());

    controls.intake().onTrue(m_robotCommands.fill());

    controls.shoot().onTrue(m_robotCommands.empty());

    controls.snowBlow().onTrue(m_robotCommands.snowBlow());

    controls.zeroHood().onTrue(m_robotCommands.zeroTurretHood());
  }

  public void configureTrimControlBindings(TrimControls controls) {
    controls.increaseFlywheelVelocity().onTrue(m_robotCommands.increaseFlywheelVelocity());
    controls.decreaseFlywheelVelocity().onTrue(m_robotCommands.decreaseFlywheelVelocity());

    controls.increaseHoodAngle().onTrue(m_robotCommands.increaseHoodAngle());
    controls.decreaseHoodAngle().onTrue(m_robotCommands.decreaseHoodAngle());

    controls.increaseVelocityCompensation()
        .onTrue(m_robotCommands.increaseVelocityCompensation());
    controls.decreaseVelocityCompensation()
        .onTrue(m_robotCommands.decreaseVelocityCompensation());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_viz.periodic();
    m_robotCommands.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void robotInit() {
    // Threads.setCurrentThreadPriority(true, 10);
  }
}
