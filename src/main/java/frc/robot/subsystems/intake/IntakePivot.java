package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

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

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO m_io;
  private final IntakePivotIOInputsAutoLogged m_inputs = new IntakePivotIOInputsAutoLogged();
  private final Trigger m_currentSpikeTrigger;
  private final LinearFilter m_currentSpikeFilter;
  private final Alert m_talonConnectionAlert = new Alert("IntakePivot TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("IntakePivot stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_setpointAngle = new LoggedTunableNumber("IntakePivot/SetpointAngleDegrees", 0);

  public IntakePivot(IntakePivotIO io) {
    m_io = io;
    m_currentSpikeFilter = LinearFilter.movingAverage(5);
    m_currentSpikeTrigger = new Trigger(this::getIsCurrentSpiking);
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/IntakePivot", tuningMode());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("IntakePivot", m_inputs);

    Logger.recordOutput("IntakePivot/currentSpike", getIsCurrentSpiking());

    boolean highCurrent = m_inputs.statorCurrent.gt(Amps.of(68));

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(highCurrent);

    m_watchdog.end("periodic");
  }

  private boolean getIsCurrentSpiking() {
    return Amps.of(m_currentSpikeFilter.calculate(m_inputs.statorCurrent.in(Amps))).gt(Amps.of(20));
  }

  public Command stow() {
    return run(() -> m_io.setPosition(Degrees.of(40)));
  }

  public Command intake() {
    return run(() -> m_io.setVoltage(Volts.of(-3)));
  }

  public Command tuningMode() {
    return run(() -> m_io.setPosition(Degrees.of(m_setpointAngle.getAsDouble())));
  }

  public Angle getAngle() {
    return m_inputs.position;
  }

  public Command setAngle(Supplier<Angle> angle) {
    return run(() -> m_io.setPosition(angle.get()));
  }

  public Command setAngle(Angle angle) {
    return setAngle(() -> angle);
  }

  public Command setVoltage(Voltage volts) {
    return run(() -> m_io.setVoltage(volts));
  }

  public Trigger getCurrentSpikeTrigger() {
    return m_currentSpikeTrigger;
  }

	public boolean isDown() {
		return getAngle().isNear(Degrees.of(0), Degrees.of(5));
	}

  public void setLowerSoftLimitEnabled(boolean enabled) {
    m_io.setLowerSoftLimitEnabled(enabled);
  }

  public Command setLowerSoftLimitEnabledCommand(boolean enabled) {
    return Commands.runOnce(() -> setLowerSoftLimitEnabled(enabled));
  }

  public void setEncoderPosition(Angle angle) {
    m_io.setEncoderPosition(angle);
  }
}
