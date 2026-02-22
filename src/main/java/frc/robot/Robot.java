package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.subsystems.indexer.TransportRoller;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeSetpoint;
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
  private ShootingParameters m_shootingParameters;

  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

  private final Feeder m_feeder;

  private final TransportRoller m_transportRoller;

  private final RobotState m_state;

  private final RobotViz m_viz;

  public Robot() {
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());
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

    m_viz = new RobotViz(m_state);

    m_autoFactory = new AutoFactory(
        m_drive::getPose,
        m_drive::resetPose,
        m_drive::followPath,
        true,
        m_drive);

    m_autoRoutines = new AutoRoutines(m_autoFactory);
    m_autoChooser = new AutoChooser("Do Nothing");

    if (Constants.kControlMode == ControlMode.Conventional) {
      var controls = new ConventionalXboxControls(0);
      configureConventionalBindings(controls);
      m_controls = controls;
    } else {
      var controls = new FourWayXboxControls(0);
      configureFourWayBindings(controls);
      m_controls = controls;
    }

    m_drive.setDefaultCommand(Constants.kUseWeirdSnakeDrive ? snakeDrive() : joyStickDrive());

    m_trimControls = new TrimXboxControls(0);
    configureTrimControlBindings(m_trimControls);

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

  private Command setIntakeSetpointCommand(IntakeSetpoint setpoint) {
    return Commands.parallel(
        m_intakePivot.setAngleCommand(setpoint.angle),
        m_intakeRoller.setVoltageCommand(setpoint.voltage));
  }

  private Command runIntakeCommand() {
    return setIntakeSetpointCommand(IntakeSetpoint.INTAKING);
  }

  private Command stowIntakeCommand() {
    return setIntakeSetpointCommand(IntakeSetpoint.STOW);
  }

  public Command joyStickDrive() {
    return Commands.run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds(
          m_controls.getDriveForward() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
          m_controls.getDriveLeft() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
          Math.copySign(m_controls.getDriveRotation() * m_controls.getDriveRotation() * Constants.kAngularMaxSpeed.in(RadiansPerSecond) * Constants.kAngluarSpeedMultiplier, m_controls.getDriveRotation()));

      m_drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));
    }, m_drive);
  }

  public Command snakeDrive() {

    return Commands.run(() -> {
      final PIDController headingController = new PIDController(100, 0, 0);
      final boolean snakeDriveActive = !(Math.abs(m_controls.getDriveRotation()) > 0);

      ChassisSpeeds speeds;
      if (snakeDriveActive) {
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        speeds = new ChassisSpeeds(
            m_controls.getDriveForward() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
            m_controls.getDriveLeft() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
            0);

        Angle headingAngle = Radians.of(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + Math.PI);

        if (Math.abs(speeds.vyMetersPerSecond) > 0.1 || Math.abs(speeds.vxMetersPerSecond) > 0.1) {
          speeds.omegaRadiansPerSecond = headingController.calculate(m_drive.getPose().getRotation().getRadians(),
              headingAngle.in(Radians));
        }
      } else {
        speeds = new ChassisSpeeds(
            m_controls.getDriveForward() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
            m_controls.getDriveLeft() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.kLinearSpeedMultiplier,
            Math.copySign(m_controls.getDriveRotation() * m_controls.getDriveRotation() * Constants.kAngularMaxSpeed.in(RadiansPerSecond) * Constants.kAngluarSpeedMultiplier, m_controls.getDriveRotation()));

      }

      m_drive.setControl(new SwerveRequest.FieldCentric()
          .withVelocityX(speeds.vxMetersPerSecond)
          .withVelocityY(speeds.vyMetersPerSecond)
          .withRotationalRate(speeds.omegaRadiansPerSecond));

      headingController.close();
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
