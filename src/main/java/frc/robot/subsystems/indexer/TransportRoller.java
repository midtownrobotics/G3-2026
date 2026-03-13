package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class TransportRoller extends SubsystemBase {
  private final TransportRollerIO m_io;
  private final TransportRollerIOInputsAutoLogged m_inputs = new TransportRollerIOInputsAutoLogged();
  private final Alert m_talonConnectionAlert = new Alert("TransportRoller TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("TransportRoller stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("TransportRoller/kP", 0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("TransportRoller/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("TransportRoller/kD", 0);

  public TransportRoller(TransportRollerIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("TransportRoller", m_inputs);

    boolean highCurrent = m_inputs.statorCurrent.gt(Amps.of(68));
    boolean notMoving = Math.abs(m_inputs.velocity.in(RPM)) < 120;

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(highCurrent && notMoving);

    LoggedTunableNumber.ifChanged(
        hashCode(), values -> m_io.setPID(values[0], values[1], values[2]), m_kP, m_kI, m_kD);

    m_watchdog.end("periodic");
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return Commands.run(() -> m_io.setSpeed(speed), this);
  }

  public Command setVoltageCommand(Voltage volts) {
    return Commands.run(() -> m_io.setVoltage(volts), this);
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> m_io.stop(), this);
  }
}
