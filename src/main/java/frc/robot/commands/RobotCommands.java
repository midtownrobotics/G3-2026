package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterState;
import frc.robot.constants.Constants;
import frc.robot.controls.Controls;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;

public class RobotCommands {
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Feeder m_feeder;
  private final Indexer m_indexer;
  private final Flywheel m_shooter;
  private final Hood m_hood;
  private final RobotState m_state;
  private final DriveCommands m_driveCommands;

  public RobotCommands(
      Drive drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Turret turret,
      Feeder feeder,
      Vision vision,
      Indexer indexer,
      Flywheel shooter,
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
            hoodAndFlywheelTrackShootingParamters(), runIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("snowBlow");
  }

  public Command autoAimAndPrepareShootTeleop() {
    return Commands
        .parallel(prepareShoot(),
            Commands.either(autoAimWithDrivetrainForTeleop(), driveCommand(),
                m_state::isAutoAimAndFixedTurretModeEnabled))
        .withName("autoAimAndPrepareShootTeleop");
  }

	public Command stowHood() {
		return m_hood.setAngleCommand(Degrees.zero());
	}

  public Command hoodAndFlywheelTrackShootingParamters() {
    return Commands.parallel(
        m_hood.setAngleCommand(() -> m_state.getShootingParameters().getParameters().hoodAngle()),
        m_shooter.setSpeedCommandWithFeedForward(() -> m_state.getShootingParameters().getParameters().flywheelVelocity()))
        .withName("hoodAndFlywheelTrackShootingParamters");
  }

	public Command hoodAndFlywheelTrackShootingParamtersm13() {
    return Commands.parallel(
        m_hood.setAngleCommand(() -> m_state.getShootingParameters().getParameters().hoodAngle()),
        m_shooter.setSpeedCommand(() -> m_state.getShootingParameters().getParameters().flywheelVelocity()))
        .withName("hoodAndFlywheelTrackShootingParamters");
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
    return Commands.parallel(hoodAndFlywheelTrackShootingParamters(),
        m_state.setShooterStateCommand(ShooterState.kRev));
  }

  public Command shootShooterCommand() {
    return Commands.parallel(hoodAndFlywheelTrackShootingParamters(),
        m_state.setShooterStateCommand(ShooterState.kShoot));
  }

	public Command startShootingCommand() {
		return m_state.setShooterStateCommand(ShooterState.kShoot);
	}

	public Command stopShootingCommand() {
		return m_state.setShooterStateCommand(ShooterState.kIdle);
	}

  public Command stopShooterCommand() {
    return Commands.parallel(m_shooter.stop(), m_state.setShooterStateCommand(ShooterState.kIdle));
  }

  public Command shakeIntake() {
    return Commands.repeatingSequence(m_intakePivot.setAngle(Degrees.of(45)).withTimeout(0.2),
        m_intakePivot.setAngle(Degrees.of(20)).withTimeout(0.2));
  }

  public Command runIntake() {
    return Commands.parallel(
        m_intakePivot.intake(),
        m_intakeRoller.intake()).withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("runIntake");
  }

  public Command reverseIntake() {
    return Commands.parallel(
        m_intakePivot.intake(),
        m_intakeRoller.reverseIntake()).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("reverseIntake");
  }

  public Command intakeDownNoRollers() {
    return Commands.parallel(
        m_intakePivot.intake(),
        m_intakeRoller.stop())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("intakeDownNoRollers");
  }

  public Command stowIntake() {
    return Commands.parallel(m_intakePivot.stow(), m_intakeRoller.stow())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("stowIntake");
  }

  public Command idle() {
    return Commands
        .parallel(m_shooter.stop(), m_feeder.stop(), stowIntake(), m_state.setShooterStateCommand(ShooterState.kIdle))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("idle");
  }

  public Command stowIntakeAndHaltTurretMovement() {
    return Commands.parallel(idle(), m_turret.stop(), zeroTurretHood().andThen(stowHood()))
        .withTimeout(Seconds.of(0.5))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("stowIntakeAndHaltTurretMovement");
  }

  public Command haltTurretAndHoodMovement() {
    return Commands.parallel(m_turret.stop(), stowHood()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command fill() {
    return Commands
        .parallel(m_shooter.stop(), m_turret.stop(), m_feeder.stop(), runIntake(), stowHood(), m_state.setShooterStateCommand(ShooterState.kIdle))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("fill");
  }

  public Command prepareShoot() {
    return Commands.parallel(hoodAndFlywheelTrackShootingParamters(), shakeIntake())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("prepareShoot");
  }

  public Command feedFuel() {
    return Commands.parallel(m_feeder.runForward(), m_indexer.runForward())
        .withName("feedFuel");
  }

  public Command defense() {
    Supplier<Boolean> turretBlockingIntake = () -> m_state.getTurretAngle().gt(Degrees.of(94))
        || m_state.getHoodAngle().gt(Degrees.of(1));
    return Commands.parallel(
        m_turret.setAngleCommand(Degrees.of(90)),
        m_hood.setAngleCommand(Degrees.zero()),
        m_feeder.stop(), m_indexer.stop(),
        m_state.setShooterStateCommand(ShooterState.kIdle),
        m_intakeRoller.stop(),
        m_intakePivot.setAngle(() -> Degrees.of(turretBlockingIntake.get() ? 40 : 65))).withName("defense");
  }

  public Command stopFeedingFuel() {
    return Commands.parallel(m_feeder.stop(), m_indexer.stop()).withName("stopFeedingFuel");
  }

  public Command reverseFeedFuel() {
    return Commands.parallel(m_feeder.runReverse(), m_indexer.runReverse()).withName("reverseFeedFuel");
  }

  public Command setPointShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommandWithFeedForward(() -> RPM.of(1800)), m_hood.setAngleCommand(Degrees.of(2)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("setPointShoot");
  }

  public Command fullFieldFeedShoot() {
    return Commands.parallel(
        m_shooter.setSpeedCommand(RPM.of(2600)), m_hood.setAngleCommand(Degrees.of(25)))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf).withName("fullFieldFeedShoot");
  }

  public Command turretTrackShootingParameters() {
    return m_turret.setAngleCommand(
        () -> m_state.isFixedTurretModeEnabled()
            ? Constants.kFixedTurretRotation
            : m_state.getShootingParameters().getParameters().turretAngle())
        .withName("turretTrackShootingParameters");
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

	public Command zeroTurretAngle() {
		return m_turret.zeroEncoderAngleCommand().ignoringDisable(true);
	}

  public Command zeroIntake() {
    return m_intakePivot.setLowerSoftLimitEnabledCommand(false)
        .andThen(m_intakePivot.setVoltage(Volts.of(-3.5)).until(m_intakePivot.getCurrentSpikeTrigger())
            .withTimeout(Seconds.of(4)))
        .finallyDo(() -> {
          m_intakePivot.setLowerSoftLimitEnabled(true);
          m_intakePivot.setEncoderPosition(Degrees.zero());
        }).withName("zeroIntake");
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

  public Command increaseTurretAngle() {
    return Commands.runOnce(m_state.getShootingParameters()::increaseTurretAngle);
  }

  public Command decreaseTurretAngle() {
    return Commands.runOnce(m_state.getShootingParameters()::decreaseTurretAngle);
  }

  public Command driveStrightRobotRelative() {
    return m_driveCommands.straightDrive();
  }
}
