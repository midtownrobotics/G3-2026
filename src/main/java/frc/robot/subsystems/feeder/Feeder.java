package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Feeder extends SubsystemBase {
  private final FeederIO m_io;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final LinearFilter m_fuelSensorFilter;
  private final Alert m_talonConnectionAlert = new Alert("Feeder TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Feeder stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Feeder/kP", 0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Feeder/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Feeder/kD", 0);
  private final LoggedTunableNumber m_speedSetpoint = new LoggedTunableNumber("Feeder/SpeedSetpointRPM", 0);

  public Feeder(FeederIO io) {
    m_io = io;
    m_fuelSensorFilter = LinearFilter.movingAverage(5);
    m_watchdog = new Watchdawg(getClass());
  }

  private boolean getFuelSensorTripped() {
    return m_fuelSensorFilter.calculate(m_inputs.fuelSensorDistance.in(Meters)) < Inches.of(5).baseUnitMagnitude();
  }

  public Trigger fuelSensorTripped() {
    return new Trigger(this::getFuelSensorTripped).debounce(Milliseconds.of(100).in(Seconds));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);

    boolean highCurrent = m_inputs.statorCurrent.gt(Amps.of(30));
    boolean notMoving = m_inputs.velocity.abs(RPM) < 120;

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(highCurrent && notMoving);

    LoggedTunableNumber.ifChanged(
        hashCode(), values -> m_io.setPID(values[0], values[1], values[2]), m_kP, m_kI, m_kD);

    Logger.recordOutput("Feeder/sensorTripped", getFuelSensorTripped());
    Logger.recordOutput("Feeder/sensorDistance", m_inputs.fuelSensorDistance);

    m_watchdog.end("periodic");
  }

  public Command setSpeedCommand(AngularVelocity angularVelocity) {
    return run(() -> m_io.setSpeed(angularVelocity));
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> angularVelocity) {
    return run(() -> m_io.setSpeed(angularVelocity.get()));
  }

  public Command setVoltageCommand(Supplier<Voltage> voltage) {
    return run(() -> m_io.setVoltage(voltage.get()));
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> m_io.setVoltage(voltage));
  }

  public Command tuningMode() {
    return setSpeedCommand(() -> RPM.of(m_speedSetpoint.getAsDouble()));
  }
}
