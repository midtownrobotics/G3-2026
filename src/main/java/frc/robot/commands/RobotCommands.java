package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Logger;
import frc.robot.RobotState;
import frc.robot.ShootingParameters;
import frc.robot.constants.FieldConstants;
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
            ShootingParameters shootingParameters,
            RobotState state) {
        m_drive = drive;
        m_intakePivot = intakePivot;
        m_intakeRoller = intakeRoller;
        m_turret = turret;
        m_feeder = feeder;
        m_transportRoller = transportRoller;
        m_shooter = shooter;
        m_hood = hood;
        m_shootingParameters = shootingParameters;
        m_log = new Logger(getClass());
        m_state = state;
    }

    public Command shootCommand() {
        return Commands.parallel(
                stowIntake(),
                m_shooter.setSpeedCommand(() -> m_shootingParameters.getParameters().flywheelVelocity()),
                m_feeder.setVoltageCommand(Volts.of(-10)),
                DriveCommands.rotateRobot(m_drive, () -> m_shootingParameters.getTargetRotation(() -> getTarget())));
    }

    public Command runIntake() {
        return Commands.parallel(
                SubsystemCommands.intakeRunPosition(m_intakePivot),
                SubsystemCommands.runIntakeRollers(m_intakeRoller));
    }

    public Command stowIntake() {
        return Commands.parallel(
                SubsystemCommands.intakeStowPosition(m_intakePivot),
                SubsystemCommands.stopIntakeRollers(m_intakeRoller));
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
}
