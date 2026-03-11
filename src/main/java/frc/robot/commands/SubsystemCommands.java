package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.TransportRoller;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;

public class SubsystemCommands {
    protected static Command intakeStartPosition(IntakePivot intakePivot) {
        return intakePivot.setAngleCommand(Degrees.of(72));
    }

    protected static Command intakeStowPosition(IntakePivot intakePivot) {
        return intakePivot.setAngleCommand(Degrees.of(30));
    }

    protected static Command intakeRunPosition(IntakePivot intakePivot) {
        return intakePivot.setAngleCommand(Degrees.of(0));
    }

    protected static Command runIntakeRollers(IntakeRoller intakeRoller) {
        return intakeRoller.setVoltageCommand(Volts.of(7));
    }

    protected static Command stopIntakeRollers(IntakeRoller intakeRoller) {
        return intakeRoller.setVoltageCommand(Volts.of(2));
    }

    protected static Command runFeeders(Feeder feeder) {
        return feeder.setVoltageCommand(Volts.of(10));
    }

    protected static Command runFeedersReverse(Feeder feeder) {
        return feeder.setVoltageCommand(Volts.of(-10));
    }

    protected static Command stopFeeders(Feeder feeder) {
        return feeder.setVoltageCommand(Volts.of(0));
    }

    protected static Command runTransportRollers(TransportRoller transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(-10));
    }

    protected static Command runTransportRollersReverse(TransportRoller transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(3));
    }

    protected static Command stopTransportRollers(TransportRoller transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(0));
    }
}
