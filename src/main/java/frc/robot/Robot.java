package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlMode;
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
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final DriveControls m_controls;
  private final Optional<TrimControls> m_trimControls;

  private final CommandSwerveDrivetrain m_drive;
  private final Vision m_vision;

  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;

  private final Turret m_turret;
  private final Hood m_hood;
  private final Shooter m_shooter;

  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

  private final Feeder m_feeder;

  private final RobotState m_state;

  private final RobotViz m_viz;
  private final ShootingParameters m_shootingParameters;

  public Robot() {
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());
    DataLogManager.start();
    Epilogue.bind(this);

    m_drive = TunerConstants.createDrivetrain();
    m_feeder = new Feeder();
    m_intakePivot = new IntakePivot();
    m_intakeRoller = new IntakeRoller();
    m_turret = new Turret(0, 0);
    m_hood = new Hood(0, 0);
    m_shooter = new Shooter(0, 0, 0, 0);

    Camera rearFacingRightCamera = new Camera("rearFacingRightCamera", new Transform3d());
    Camera frontFacingRightCamera = new Camera("frontFacingRightCamera", new Transform3d());
    Camera rearFacingLeftCamera = new Camera("rearFacingLeftCamera", new Transform3d());
    Camera frontFacingLeftCamera = new Camera("frontFacingLeftCamera", new Transform3d());

    m_vision = new Vision(
        (observation) -> m_drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp()),
        m_drive::getPose,
        rearFacingRightCamera,
        frontFacingRightCamera,
        rearFacingLeftCamera,
        frontFacingLeftCamera);

    m_state = new RobotState(m_drive, m_intakePivot, m_turret, m_hood, m_shooter);

    m_viz = new RobotViz(m_state);

    m_shootingParameters = new ShootingParameters(m_state, () -> FieldConstants.kHubPosition.toTranslation2d());

    m_autoFactory = new AutoFactory(
        m_drive::getPose, // A function that returns the current robot pose
        m_drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
        m_drive::followPath, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
        m_drive // The drive subsystem
    );

    m_autoRoutines = new AutoRoutines(m_autoFactory);
    m_autoChooser = new AutoChooser("Do Nothing");

    if (Constants.kControlMode == ControlMode.Conventional) {
      var controls = new ConventionalXboxControls(0);
      configureConventionalBindings(controls);
      m_controls = controls;
      m_drive.setDefaultCommand(joyStickDrive());
    } else {
      var controls = new FourWayXboxControls(0);
      configureFourWayBindings(controls);
      m_controls = controls;
      m_drive.setDefaultCommand(joyStickDrive());
    }

    if (Constants.kUseTrimControls) {
      m_trimControls = Optional.of(new TrimXboxControls(0));
      configureTrimControlBindings(m_trimControls.get());
    } else {
      m_trimControls = Optional.empty();
    }

    generateAutoChooser();
  }

  private void generateAutoChooser() {
    m_autoChooser.addRoutine("Example Movement", m_autoRoutines::exampleMovementAuto);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    new Trigger(DriverStation::isAutonomousEnabled).whileTrue(m_autoChooser.selectedCommandScheduler());
  }

  public void configureConventionalBindings(ConventionalControls controls) {
    controls.shoot().onTrue(m_shooter.setSpeedCommand(RPM.of(6000))).onFalse(m_shooter.setSpeedCommand(RPM.of(0)));
    controls.intake().onTrue(runIntakeCommand()).onFalse(stowIntakeCommand());
  }

  public void configureFourWayBindings(FourWayControls controls) {
    controls.idle().onTrue(Commands.parallel(
        stowIntakeCommand(),
        m_shooter.setSpeedCommand(RPM.of(0))));

    controls.fill().onTrue(Commands.parallel(
        runIntakeCommand(),
        m_shooter.setSpeedCommand(RPM.of(0))));

    controls.empty().onTrue(Commands.parallel(
        stowIntakeCommand(),
        m_shooter.setSpeedCommand(RPM.of(6000))));

    controls.snowBlow().onTrue(Commands.parallel(
        runIntakeCommand(),
        m_shooter.setSpeedCommand(RPM.of(6000))));
  }

  public void configureTrimControlBindings(TrimControls controls) {
    controls.increaseFlywheelVelocity().onTrue(Commands.runOnce(m_shootingParameters::increaseFlywheelVelocity));
    controls.decreaseFlywheelVelocity().onTrue(Commands.runOnce(m_shootingParameters::decreaseFlywheelVelocity));

    controls.increaseHoodAngle().onTrue(Commands.runOnce(m_shootingParameters::increaseHoodAngle));
    controls.decreaseHoodAngle().onTrue(Commands.runOnce(m_shootingParameters::decreaseHoodAngle));

    controls.increaseVelocityCompensation()
        .onTrue(Commands.runOnce(m_shootingParameters::increaseVelocityCompensation));
    controls.decreaseVelocityCompensation()
        .onTrue(Commands.runOnce(m_shootingParameters::decreaseVelocityCompensation));
  }

  private Command setIntakeGoalCommand(IntakeGoal goal) {
    return Commands.parallel(
        m_intakePivot.setAngleCommand(goal.angle),
        m_intakeRoller.setVoltageCommand(goal.voltage));
  }

  private Command runIntakeCommand() {
    return setIntakeGoalCommand(IntakeGoal.INTAKING);
  }

  private Command stowIntakeCommand() {
    return setIntakeGoalCommand(IntakeGoal.STOW);
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

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_viz.periodic();
    m_shootingParameters.periodic();

    DogLog.log("Autonomous", DriverStation.isAutonomousEnabled());
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
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
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
}
