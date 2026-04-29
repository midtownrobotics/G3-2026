package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Watchdawg;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO m_io;
  private final IntakeRollerIOInputsAutoLogged m_inputs = new IntakeRollerIOInputsAutoLogged();
  private final Alert m_talonConnectionAlert = new Alert("IntakeRoller TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("IntakeRoller stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  public IntakeRoller(IntakeRollerIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    m_io.updateInputs(m_inputs);
    Logger.processInputs("IntakeRoller", m_inputs);

    boolean highCurrent = m_inputs.statorCurrent.gt(Amps.of(68));
    boolean notMoving = m_inputs.velocity.abs(RPM) < 120; // ~2 RPS in RPM

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(highCurrent && notMoving);

    m_watchdog.end("periodic");
  }

  public Command intake() {
    return run(() -> m_io.setVoltage(Volts.of(7)));
  }

  public Command reverseIntake() {
    return run(() -> m_io.setVoltage(Volts.of(-3)));
  }

  public Command stow() {
    return run(() -> m_io.setVoltage(Volts.of(4)));
  }

  public Command stop() {
    return run(() -> m_io.stop());
  }
}
