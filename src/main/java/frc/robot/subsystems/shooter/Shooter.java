package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
  private final Alert m_talon1ConnectionAlert = new Alert("Shooter TalonFX motor 1 is not connected",
      AlertType.kWarning);
  private final Alert m_talon2ConnectionAlert = new Alert("Shooter TalonFX motor 2 is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert1 = new Alert("Shooter motor 1 stalling", AlertType.kWarning);
  private final Alert m_stallAlert2 = new Alert("Shooter motor 2 stalling", AlertType.kWarning);

  private final Watchdawg m_watchdog;
  private final Trigger m_isNearSetpointTrigger;

  private final LoggedTunableNumber m_shooterSetpointSpeed = new LoggedTunableNumber(
      "Shooter/SetpointRPM", 0);

  public Shooter(ShooterIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Shooter", tuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(RPM.of(15)));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    boolean motor1HighCurrent = m_inputs.statorCurrent1.gt(Amps.of(68));
    boolean motor1NotMoving = Math.abs(m_inputs.velocity1.in(RotationsPerSecond)) < 2;
    boolean motor2HighCurrent = m_inputs.statorCurrent2.gt(Amps.of(68));
    boolean motor2NotMoving = Math.abs(m_inputs.velocity2.in(RotationsPerSecond)) < 2;

    m_talon1ConnectionAlert.set(!m_inputs.motor1Connected);
    m_talon2ConnectionAlert.set(!m_inputs.motor2Connected);
    m_stallAlert1.set(motor1HighCurrent && motor1NotMoving);
    m_stallAlert2.set(motor2HighCurrent && motor2NotMoving);

    Logger.recordOutput("Shooter/isNearSetpoint", isNearSetpointTrigger().getAsBoolean());

    m_watchdog.end("periodic");
  }

  public AngularVelocity getSpeed() {
    return m_inputs.velocity;
  }

  public AngularVelocity getSetpointSpeed() {
    return m_inputs.setpoint;
  }

  public boolean isNearSetpoint(AngularVelocity tolerance) {
    return getSpeed().isNear(getSetpointSpeed(), tolerance);
  }

  public Trigger isNearSetpointTrigger() {
    return m_isNearSetpointTrigger;
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return run(() -> m_io.setSpeed(speed));
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> speedSupplier) {
    return run(() -> m_io.setSpeed(speedSupplier.get()));
  }

  public Command stop() {
    return run(() -> m_io.stop());
  }

  public Command tuningMode() {
    return setSpeedCommand(() -> RPM.of(m_shooterSetpointSpeed.getAsDouble()));
  }
}
