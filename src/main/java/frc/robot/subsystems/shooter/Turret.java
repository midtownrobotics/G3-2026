package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Turret extends SubsystemBase {
  private final TurretIO m_io;
  private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();
  private final Alert m_talonConnectionAlert = new Alert("Turret TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Turret motor stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;
  private final Trigger m_isNearSetpointTrigger;

  private final LoggedTunableNumber m_turretSetpointAngleDegrees = new LoggedTunableNumber(
      "Turret/SetpointDegrees", 0);

  public Turret(TurretIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Turret", tuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(Degrees.of(5)));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Turret", m_inputs);

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(m_inputs.statorCurrent.gt(Amps.of(68)));

    Logger.recordOutput("Turret/angle", getAngle());
    Logger.recordOutput("Turret/isNearSetpoint", isNearSetpointTrigger().getAsBoolean());

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

  public Command setAngleCommand(Angle angle) {
    return run(() -> m_io.setPosition(angle));
  }

	public Command setEncoderAngleCommand(Angle angle) {
		return Commands.runOnce(() -> m_io.setEncoderPosition(angle));
	}

	public Command zeroEncoderAngleCommand() {
		return setEncoderAngleCommand(Degrees.of(90));
	}

  public Command setAngleCommand(Supplier<Angle> angle) {
    return run(() -> m_io.setPosition(angle.get()));
  }

  public Command stop() {
    return run(() -> m_io.stop());
  }

  public Command tuningMode() {
    return setAngleCommand(() -> Degrees.of(m_turretSetpointAngleDegrees.getAsDouble()));
  }
}
