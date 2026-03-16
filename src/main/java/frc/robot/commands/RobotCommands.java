package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.controls.DriveControls;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;

public class RobotCommands {
  private final CommandSwerveDrivetrain m_drive;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Indexer m_indexer;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final RobotState m_state;
  private final DriveControls m_controls;

  public RobotCommands(
      CommandSwerveDrivetrain drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      Indexer indexer,
      Shooter shooter,
      Hood hood,
      RobotState state,
      DriveControls controls) {
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_controls = controls;
    m_indexer = indexer;
    m_shooter = shooter;
    m_hood = hood;
    m_state = state;
  }

  public Command snowBlow() {
    return Commands.parallel(autoAimForTeleop(), runFlywheelCommand(), runIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command autoAimAndPrepareShootTeleop() {
    return Commands.parallel(prepareShoot(), autoAimForTeleop());
  }

  public Command autoAimAndPrepareShootAutonomous() {
    return Commands.parallel(prepareShoot(), autoAimForAutonomous());
  }

  private Command runFlywheelCommand() {
    return m_shooter.setSpeedCommand(() -> m_state.getShootingParameters().getParameters().flywheelVelocity());
  }

  public Command autoAimForTeleop() {
    return DriveCommands.rotateRobot(
        m_drive,
        () -> m_state.getShootingParameters().getTargetRobotRotation(),
        m_controls::getDriveForward,
        m_controls::getDriveLeft);
  }

  public Command autoAimForAutonomous() {
    return DriveCommands.rotateRobot(m_drive, m_state.getShootingParameters()::getTargetRobotRotation);
  }

  private Command intakeStowPosition() {
    return m_intakePivot.setAngleCommand(Degrees.of(30));
  }

  private Command intakeRunPosition() {
    return m_intakePivot.setAngleCommand(Degrees.of(5));
  }

  private Command runIntakeRollers() {
    return m_intakeRoller.setVoltageCommand(Volts.of(7));
  }

  private Command stopIntakeRollers() {
    return m_intakeRoller.setVoltageCommand(Volts.of(2));
  }

  public Command runIntake() {
    return Commands.parallel(
        intakeRunPosition(),
        runIntakeRollers());
  }

  public Command stowIntake() {
    return Commands.parallel(intakeStowPosition(), stopIntakeRollers())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command idle() {
    return Commands.parallel(m_shooter.setSpeedCommand(RPM.of(0)), stowIntake(), stopFeedingFuel())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command stopFlywheel() {
    return Commands.parallel(m_shooter.setSpeedCommand(RPM.of(0)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command fill() {
    return Commands.parallel(stopFlywheel(), runIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command prepareShoot() {
    return Commands.parallel(runFlywheelCommand(), stowIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  private Command runFeeder() {
    return m_feeder.setVoltageCommand(Volts.of(10));
  }

  private Command runFeederReverse() {
    return m_feeder.setVoltageCommand(Volts.of(-10));
  }

  private Command stopFeeder() {
    return m_feeder.setVoltageCommand(Volts.of(0));
  }

  private Command runIndexer() {
    return m_indexer.setVoltageCommand(Volts.of(10));
  }

  private Command stopIndexer() {
    return m_indexer.setVoltageCommand(Volts.of(0));
  }

  private Command runIndexerReverse() {
    return m_indexer.setVoltageCommand(Volts.of(-3));
  }

  public Command feedFuel() {
    return Commands.parallel(runFeeder(), runIndexer());
  }

  public Command stopFeedingFuel() {
    return Commands.parallel(stopFeeder(), stopIndexer());
  }

  public Command reverseFeedFuel() {
    return Commands.parallel(runFeederReverse(), runIndexerReverse(), driveCommand());
  }

  public Command revShooter() {
    return m_shooter.setSpeedCommand(RPM.of(1800));
  }

  public Command setPointShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommand(RPM.of(1800)), m_hood.setAngleCommand(Degrees.of(2)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command fullFieldFeedShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommand(RPM.of(2600)), m_hood.setAngleCommand(Degrees.of(25)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command alignHood() {
    return m_hood.setAngleCommand(() -> m_state.getShootingParameters().getParameters().hoodAngle());
  }

  public Command alignTurret() {
    return m_turret.setAngleCommand(
        () -> m_state.isFixedTurretModeEnabled()
            ? Constants.kFixedTurretRotation
            : m_state.getShootingParameters().getParameters().turretAngle());
  }

  public Command driveCommand() {
    return DriveCommands.driveCommand(
        m_drive,
        m_controls::getDriveForward,
        m_controls::getDriveLeft,
        m_controls::getDriveRotation);
  }

  public Command zeroTurretHood() {
    return Commands.repeatingSequence(
        m_hood
            .setVoltage(Volts.of(-3.5))
            .until(m_hood.isNearTrigger(() -> Degrees.zero(), Degrees.of(1)))
            .withTimeout(Seconds.of(1)),
        m_hood.setEncoderAngleCommand(Degrees.of(10)))
        .withTimeout(4)
        .until(m_hood.getCurrentSpikeTrigger())
        .andThen(m_hood.zeroEncoderAngleCommand());
  }

  public Translation2d calculateFeedTarget() {
    double robotY = m_state.getRobotPose().getY();
    Translation2d hubPosition = FieldConstants.getHubPosition2d();
    double hubY = hubPosition.getY();

    double targetY = robotY;

    if (robotY > (hubY - 0.762) && robotY < (hubY + 0.762)) {
      if (robotY > hubY) {
        targetY = hubY + 1;
      } else {
        targetY = hubY - 1;
      }
    }

    return new Translation2d(hubPosition.getX(), targetY);
  }

  public Command increaseFlywheelVelocity() {
    return Commands.runOnce(m_state.getShootingParameters()::increaseFlywheelVelocity);
  }

  public Command decreaseFlywheelVelocity() {
    return Commands.runOnce(m_state.getShootingParameters()::decreaseFlywheelVelocity);
  }

  public Command increaseHoodAngle() {
    return Commands.runOnce(m_state.getShootingParameters()::increaseHoodAngle);
  }

  public Command decreaseHoodAngle() {
    return Commands.runOnce(m_state.getShootingParameters()::decreaseHoodAngle);
  }

  public Command increaseVelocityCompensation() {
    return Commands.runOnce(m_state.getShootingParameters()::increaseVelocityCompensation);
  }

  public Command decreaseVelocityCompensation() {
    return Commands.runOnce(m_state.getShootingParameters()::decreaseVelocityCompensation);
  }
}
