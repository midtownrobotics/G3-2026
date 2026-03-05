package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Logger;
import frc.robot.RobotState;
import frc.robot.ShootingParameters;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.controls.DriveControls;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.TransportRoller;
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
    private final TransportRoller m_transportRoller;
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final ShootingParameters m_shootingParameters;
    private final Logger m_log;
    private final RobotState m_state;
    private final DriveControls m_driveControls;
    private final Trigger m_parametersHasShot;

    public RobotCommands(
            CommandSwerveDrivetrain drive,
            IntakePivot intakePivot,
            IntakeRoller intakeRoller,
            Turret turret,
            Feeder feeder,
            Vision vision,
            TransportRoller transportRoller,
            Shooter shooter,
            Hood hood,
            RobotState state,
            DriveControls driveControls) {
        m_drive = drive;
        m_intakePivot = intakePivot;
        m_intakeRoller = intakeRoller;
        m_turret = turret;
        m_feeder = feeder;
        m_transportRoller = transportRoller;
        m_shooter = shooter;
        m_hood = hood;
        m_state = state;
        m_driveControls = driveControls;
        m_log = new Logger(getClass());
        m_shootingParameters = new ShootingParameters(m_state, this::getTarget);
        m_parametersHasShot = new Trigger(() -> !m_shootingParameters.getParameters().noShot())
                .and(RobotModeTriggers.autonomous().negate());
    }

    public Trigger parametersHasShot() {
        return m_parametersHasShot;
    }

    private Command fixedTurretShootCommand() {
        return Commands.parallel(
                m_shooter.setSpeedCommand(() -> m_shootingParameters.getParameters().flywheelVelocity()),
                m_feeder.setVoltageCommand(Volts.of(-10)),
                DriveCommands.rotateRobot(m_drive, () -> m_shootingParameters.getTargetRotation(),
                        m_driveControls::getDriveForward, m_driveControls::getDriveLeft));
    }

    private Command movingTurretShootCommand() {
        return Commands.parallel(
                m_shooter.setSpeedCommand(() -> m_shootingParameters.getParameters().flywheelVelocity()),
                m_feeder.setVoltageCommand(Volts.of(-10)),
                DriveCommands.driveCommand(m_drive, m_driveControls::getDriveForward,
                        m_driveControls::getDriveLeft, m_driveControls::getDriveRotation));
    }

    public Command shoot() {
        return (Constants.kUseFixedTurretMode ? fixedTurretShootCommand() : movingTurretShootCommand())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command runIntake() {
        return Commands.parallel(
                SubsystemCommands.intakeRunPosition(m_intakePivot),
                SubsystemCommands.runIntakeRollers(m_intakeRoller));
    }

    public Command stowIntake() {
        return Commands.parallel(
                SubsystemCommands.intakeStowPosition(m_intakePivot),
                SubsystemCommands.stopIntakeRollers(m_intakeRoller))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command idle() {
        return Commands.parallel(
                m_shooter.setSpeedCommand(RPM.of(0)),
                stowIntake())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command stopShooting() {
        return Commands.parallel(
                m_shooter.setSpeedCommand(RPM.of(0))).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command fill() {
        return Commands.parallel(
                m_shooter.setSpeedCommand(RPM.of(0)),
                runIntake())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command snowBlow() {
        return Commands.parallel(
                shoot(),
                runIntake()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command empty() {
        return Commands.parallel(
                shoot(),
                stowIntake()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    private Command runFeeders() {
        return SubsystemCommands.runFeeders(m_feeder);
    }

    private Command stopFeeders() {
        return SubsystemCommands.stopFeeders(m_feeder);
    }

    private Command runTransportRollers() {
        return SubsystemCommands.runTransportRollers(m_transportRoller);
    }

    private Command stopTransportRollers() {
        return SubsystemCommands.stopTransportRollers(m_transportRoller);
    }

    public Command feedFuel() {
        return Commands.parallel(runFeeders(), runTransportRollers());
    }

    public Command stopFeedingFuel() {
        return Commands.parallel(stopFeeders(), stopTransportRollers());
    }

    public Command revShooter() {
        return m_shooter.setSpeedCommand(RPM.of(1800));
    }

    public Command alignHood() {
        return m_hood.setAngleCommand(() -> m_shootingParameters.getParameters().hoodAngle());
    }

    public Command alignTurret() {
        return Constants.kUseFixedTurretMode ? m_turret.setAngleCommand(Constants.kFixedTurretRotation)
                : m_turret.setAngleCommand(() -> m_shootingParameters.getParameters().turretAngle());
    }

    public Command driveCommand() {
        return DriveCommands.driveCommand(m_drive, m_driveControls::getDriveForward,
                m_driveControls::getDriveLeft, m_driveControls::getDriveRotation);
    }

    public Command zeroTurretHood() {
        return Commands.repeatingSequence(
                m_hood.setVoltage(Volts.of(-1.5)).until(m_hood.isNearTrigger(() -> Degrees.zero(), Degrees.of(1)))
                        .withTimeout(Seconds.of(1)),
                m_hood.setEncoderAngleCommand(Degrees.of(10))).withTimeout(4).until(m_hood.getCurrentSpikeTrigger())
                .andThen(m_hood.zeroEncoderAngleCommand());
    }

    public void periodic() {
        m_shootingParameters.periodic();
    }

    private Translation2d getTarget() {
        Translation2d target = calculateTarget();
        m_log.log("target", new Pose2d(target, new Rotation2d()));
        return target;
    }

    private Translation2d calculateTarget() {
        if (m_state.inAllianceZone()) {
            return FieldConstants.getHubPosition2d();
        }

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double robotY = m_state.getRobotPose().getY();
        double hubY = FieldConstants.getHubPosition2d().getY();

        if (robotY > (hubY - 0.762) && robotY < (hubY + 0.762)) {
            if (robotY > hubY) {
                return new Translation2d(FieldConstants.getHubPosition2d().getX(), hubY + 1);
            }
            return new Translation2d(FieldConstants.getHubPosition2d().getX(), hubY - 1);
        }

        return switch (alliance) {
            case Blue -> new Translation2d(FieldConstants.getHubPosition2d().getX(), robotY);
            case Red -> new Translation2d(FieldConstants.getHubPosition2d().getMeasureX(),
                    m_state.getRobotPose().getMeasureY());
        };
    }

    public Command increaseFlywheelVelocity() {
        return Commands.runOnce(m_shootingParameters::increaseFlywheelVelocity);
    }

    public Command decreaseFlywheelVelocity() {
        return Commands.runOnce(m_shootingParameters::decreaseFlywheelVelocity);
    }

    public Command increaseHoodAngle() {
        return Commands.runOnce(m_shootingParameters::increaseHoodAngle);
    }

    public Command decreaseHoodAngle() {
        return Commands.runOnce(m_shootingParameters::decreaseHoodAngle);
    }

    public Command increaseVelocityCompensation() {
        return Commands.runOnce(m_shootingParameters::increaseVelocityCompensation);
    }

    public Command decreaseVelocityCompensation() {
        return Commands.runOnce(m_shootingParameters::decreaseVelocityCompensation);
    }
}
