package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Hood extends SubsystemBase {
  private final HoodIO m_io;
  private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
  private final Trigger m_currentSpikeTrigger;
  private final LinearFilter m_currentSpikeFilter;
  private final Alert m_talonConnectionAlert = new Alert("Hood TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Hood stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;
  private final Trigger m_isNearSetpointTrigger;

  private final LoggedTunableNumber m_setpointAngle = new LoggedTunableNumber("Hood/SetpointAngleDegrees", 0);

  public Hood(HoodIO io) {
    m_io = io;
    m_currentSpikeFilter = LinearFilter.movingAverage(5);
    m_currentSpikeTrigger = new Trigger(this::getIsCurrentSpiking);
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Hood", tuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(Degrees.of(1)));
  }

  private boolean getIsCurrentSpiking() {
    return Amps.of(m_currentSpikeFilter.calculate(m_inputs.statorCurrent.in(Amps))).gt(Amps.of(20));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Hood", m_inputs);

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(m_inputs.statorCurrent.gt(Amps.of(30)));

    Logger.recordOutput("Hood/currentSpike", getIsCurrentSpiking());
    Logger.recordOutput("Hood/isNearSetpoint", isNearSetpointTrigger().getAsBoolean());

    m_watchdog.end("periodic");
  }

  public Angle getAngle() {
    return m_inputs.position;
  }

  public Angle getSetpointAngle() {
    return m_inputs.setpoint;
  }

  public boolean isNearSetpoint(Angle tolerance) {
    return getAngle().isNear(getSetpointAngle(), tolerance);
  }

  public Trigger isNearSetpointTrigger() {
    return m_isNearSetpointTrigger;
  }

  public Command setVoltage(Voltage volts) {
    return run(() -> m_io.setVoltage(volts));
  }

  public Trigger getCurrentSpikeTrigger() {
    return m_currentSpikeTrigger;
  }

  public Trigger isNearTrigger(Supplier<Angle> angle, Angle threshold) {
    return new Trigger(() -> getAngle().isNear(angle.get(), threshold));
  }

  public void setEncoderPosition(Angle angle) {
    m_io.setEncoderPosition(angle);
  }

  public Command setEncoderAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setEncoderPosition(angle));
  }

  public Command zeroEncoderAngleCommand() {
    return setEncoderAngleCommand(Degrees.zero());
  }

  public void setLowerSoftLimitEnabled(boolean enabled) {
    m_io.setLowerSoftLimitEnabled(enabled);
  }

  public Command setLowerSoftLimitEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> setLowerSoftLimitEnabled(enabled));
  }

  public Command setAngleCommand(Angle angle) {
    return run(() -> m_io.setPosition(angle));
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return run(() -> m_io.setPosition(angle.get()));
  }

  public Command stop() {
    return runOnce(() -> m_io.stop());
  }

  public Command tuningMode() {
    return setAngleCommand(() -> Degrees.of(m_setpointAngle.getAsDouble()));
  }
}
