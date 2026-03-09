package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
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
        return intakePivot.setAngleCommand(Degrees.of(5));
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

    protected static Command runTransportRollers(Indexer transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(-10));
    }

    protected static Command runTransportRollersReverse(Indexer transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(3));
    }

    protected static Command stopTransportRollers(Indexer transportRoller) {
        return transportRoller.setVoltageCommand(Volts.of(0));
    }
}
