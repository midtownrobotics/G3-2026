package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterState;
import frc.robot.constants.Constants;
import frc.robot.controls.Controls;
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
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Indexer m_indexer;
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final RobotState m_state;
  private final DriveCommands m_driveCommands;

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
      Controls controls) {
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_turret = turret;
    m_feeder = feeder;
    m_indexer = indexer;
    m_shooter = shooter;
    m_hood = hood;
    m_state = state;
    m_driveCommands = new DriveCommands(drive, controls::getDriveLeft, controls::getDriveForward,
        controls::getDriveRotation, m_state);

    SmartDashboard.putData("MissingShort", increaseHoodAngle());
    SmartDashboard.putData("MissingLong", decreaseHoodAngle());

  }

  public Command snowBlow() {
    return Commands
        .parallel(
            Commands.either(autoAimWithDrivetrainForTeleop(), driveCommand(),
                m_state::isAutoAimAndFixedTurretModeEnabled),
            shooterTrackShootingParamters(), runIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("snowBlow");
  }

  public Command autoAimAndPrepareShootTeleop() {
    return Commands
        .parallel(prepareShoot(),
            Commands.either(autoAimWithDrivetrainForTeleop(), driveCommand(),
                m_state::isAutoAimAndFixedTurretModeEnabled))
        .withName("autoAimAndPrepareShootTeleop");
  }

  public Command autoAimAndPrepareShootAutonomous() {
    return Commands
        .parallel(shootShooterCommand(), m_intakePivot.stow(),
            Commands.either(autoAimWithDrivetrainForAutonomous(), driveCommand(), m_state::isFixedTurretModeEnabled))
        .withName("autoAimAndPrepareShootAutonomous");
  }

  public Command shooterTrackShootingParamters() {
    return Commands
        .parallel(m_shooter.setSpeedCommand(() -> m_state.getShootingParameters().getParameters().flywheelVelocity()))
        .withName("shooterTrackShootingParamters");
  }

  public Command autoAimWithDrivetrainForTeleop() {
    return m_driveCommands.rotateRobot(() -> m_state.getShootingParameters().getTargetRobotRotation())
        .withName("autoAimForTeleop");
  }

  public Command autoAimWithDrivetrainForAutonomous() {
    return m_driveCommands.rotateRobotForAutonomous(() -> m_state.getShootingParameters().getTargetRobotRotation())
        .withName("autoAimForAutonomous");
  }

  public Command revShooterCommand() {
    return Commands.parallel(shooterTrackShootingParamters(), m_state.setShooterStateCommand(ShooterState.kRev));
  }

  public Command shootShooterCommand() {
    return Commands.parallel(shooterTrackShootingParamters(), m_state.setShooterStateCommand(ShooterState.kShoot));
  }

  public Command stopShooterCommand() {
    return Commands.parallel(m_shooter.stop(), m_state.setShooterStateCommand(ShooterState.kIdle));
  }

  public Command runIntake() {
    return Commands.parallel(
        m_intakePivot.intake(),
        m_intakeRoller.intake()).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("runIntake");
  }

  public Command stowIntake() {
    return Commands.parallel(m_intakePivot.stow(), m_intakeRoller.stow())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("stowIntake");
  }

  public Command idle() {
    return Commands.parallel(m_shooter.stop(), stowIntake(), m_state.setShooterStateCommand(ShooterState.kIdle))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("idle");
  }

  public Command stowIntakeAndHaltTurretMovement() {
    return Commands.parallel(idle(), m_turret.stop(), zeroTurretHood().andThen(m_hood.stop()))
        .withTimeout(Seconds.of(0.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("stowIntakeAndHaltTurretMovement");
  }

  public Command fill() {
    return Commands.parallel(m_shooter.stop(), runIntake(), m_state.setShooterStateCommand(ShooterState.kIdle))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("fill");
  }

  public Command prepareShoot() {
    return Commands.parallel(shooterTrackShootingParamters(), stowIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("prepareShoot");
  }

  public Command feedFuel() {
    return Commands.parallel(m_feeder.runForward(), m_indexer.runForward()).withName("feedFuel");
  }

  public Command stopFeedingFuel() {
    return Commands.parallel(m_feeder.stop(), m_indexer.stop()).withName("stopFeedingFuel");
  }

  public Command reverseFeedFuel() {
    return Commands.parallel(m_feeder.runReverse(), m_indexer.runReverse(), driveCommand()).withName("reverseFeedFuel");
  }

  public Command setPointShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommand(RPM.of(1800)), m_hood.setAngleCommand(Degrees.of(2)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("setPointShoot");
  }

  public Command fullFieldFeedShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommand(RPM.of(2600)), m_hood.setAngleCommand(Degrees.of(25)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("fullFieldFeedShoot");
  }

  public Command alignHood() {
    return m_hood.setAngleCommand(() -> m_state.getShootingParameters().getParameters().hoodAngle())
        .withName("alignHood");
  }

  public Command alignTurret() {
    return m_turret.setAngleCommand(
        () -> m_state.isFixedTurretModeEnabled()
            ? Constants.kFixedTurretRotation
            : m_state.getShootingParameters().getParameters().turretAngle())
        .withName("alignTurret");
  }

  public Command driveCommand() {
    return m_driveCommands.driveCommand().withName("driveCommand");
  }

  public Command zeroTurretHood() {
    return m_hood.setLowerSoftLimitEnabledCommand(false)
        .andThen(m_hood.setVoltage(Volts.of(-3.5)).until(m_hood.getCurrentSpikeTrigger()).withTimeout(Seconds.of(4)))
        .finallyDo(() -> {
          m_hood.setLowerSoftLimitEnabled(true);
          m_hood.setEncoderPosition(Degrees.zero());
        }).withName("zeroTurretHood");
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
