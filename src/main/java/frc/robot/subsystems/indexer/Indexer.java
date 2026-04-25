package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Indexer extends SubsystemBase {
  private final InexerIO m_io;
  private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();
  private final Alert m_talonConnectionAlert = new Alert("Indexer TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Indexer stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Indexer/kP", 0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Indexer/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Indexer/kD", 0);

	private final Trigger m_highCurrentTrigger;
	private Boolean m_highcurrent;

  public Indexer(InexerIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
		m_highCurrentTrigger = new Trigger(this::getHighCurrent).debounce(0.5);
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Indexer", m_inputs);

    m_highcurrent = m_inputs.statorCurrent.gt(Amps.of(68));
    boolean notMoving = Math.abs(m_inputs.velocity.in(RPM)) < 120;

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(getHighCurrent() && notMoving);

    LoggedTunableNumber.ifChanged(
        hashCode(), values -> m_io.setPID(values[0], values[1], values[2]), m_kP, m_kI, m_kD);

    m_watchdog.end("periodic");
  }

	public boolean getHighCurrent() {
		return m_highcurrent;
	}

  public Command runForward() {
    return run(() -> {
        if (m_highCurrentTrigger.getAsBoolean()) {
            runReverse();
        } else {
            m_io.setVoltage(Volts.of(8));  
        }
    });
}

  public Command stop() {
    return run(() -> m_io.setVoltage(Volts.of(0)));
  }

  public Command runReverse() {
    return run(() -> m_io.setVoltage(Volts.of(-3))).finallyDo(() -> m_io.setVoltage(Volts.zero()) );
  }
}
