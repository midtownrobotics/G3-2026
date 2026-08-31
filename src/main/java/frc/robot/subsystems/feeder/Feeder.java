package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Feeder extends SubsystemBase {
  private final FeederIO m_io;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final LinearFilter m_fuelSensorFilter;
  private final Trigger m_fuelSensorTrippedTrigger;
  private final Alert m_talonConnectionAlert = new Alert("Feeder TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Feeder stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Feeder/kP", 0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Feeder/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Feeder/kD", 0);
  private final LoggedTunableNumber m_speedSetpoint = new LoggedTunableNumber("Feeder/SpeedSetpointRPM", 0);
  private final LoggedTunableNumber m_voltageSetpoint = new LoggedTunableNumber("Feeder/voltageSetpoint", 0);
  private final LoggedTunableNumber m_feedVoltage = new LoggedTunableNumber("Feeder/feedVoltage", 10);

  private Distance m_filteredSensorDistance = Meters.zero();

  public Feeder(FeederIO io) {
    m_io = io;
    m_fuelSensorFilter = LinearFilter.movingAverage(4);
    m_fuelSensorTrippedTrigger = new Trigger(this::getFuelSensorTripped).debounce(0.1, DebounceType.kFalling);
    m_watchdog = new Watchdawg(getClass());

    SmartDashboard.putData("TuningModes/feeder", tuningMode());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);

    m_filteredSensorDistance = Meters.of(m_fuelSensorFilter.calculate(m_inputs.fuelSensorDistance.in(Meters)));

    Logger.recordOutput("Feeder/sensorFilteredDistance", m_filteredSensorDistance);

    boolean highCurrent = m_inputs.statorCurrent1.gt(Amps.of(30));
    boolean notMoving = m_inputs.velocity1.abs(RPM) < 120;

    m_talonConnectionAlert.set(!m_inputs.motorConnected1);
    m_stallAlert.set(highCurrent && notMoving);

    Logger.recordOutput("Feeder/sensorTripped", getFuelSensorTripped());

    m_watchdog.end("periodic");
  }

  private boolean getFuelSensorTripped() {
    return m_filteredSensorDistance.lt(Inches.of(6.5));
  }

  public Trigger fuelSensorTripped() {
    return m_fuelSensorTrippedTrigger;
  }

  public Command setSpeedCommand(AngularVelocity angularVelocity) {
    return run(() -> m_io.setSpeed(angularVelocity)).finallyDo(() -> m_io.stop());
  }

  public Command runForward() {
    return run(() -> m_io.setVoltage(Volts.of(m_feedVoltage.get()))).finallyDo(() -> m_io.stop());
  }

  public Command stop() {
    return runOnce(() -> m_io.stop());
  }

  public Command runReverse() {
    return run(() -> m_io.setVoltage(Volts.of(-10))).finallyDo(() -> m_io.stop());
  }

  public Command tuningMode() {
    return run(() -> m_io.setVoltage(Volts.of(m_voltageSetpoint.get())));
  }
}
