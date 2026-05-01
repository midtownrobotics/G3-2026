package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO m_io;
  private final FlywheelIOInputsAutoLogged m_inputs = new FlywheelIOInputsAutoLogged();
  private final Alert m_talon1ConnectionAlert = new Alert("Flywheel TalonFX motor 1 is not connected",
      AlertType.kWarning);
  private final Alert m_talon2ConnectionAlert = new Alert("Flywheel TalonFX motor 2 is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert1 = new Alert("Flywheel motor 1 stalling", AlertType.kWarning);
  private final Alert m_stallAlert2 = new Alert("Flywheel motor 2 stalling", AlertType.kWarning);

  private final Watchdawg m_watchdog;
  private final Trigger m_isNearSetpointTrigger;

  private final LoggedTunableNumber m_shooterSetpointSpeed = new LoggedTunableNumber(
      "Flywheel/SetpointRPM", 0);

  public Flywheel(FlywheelIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Flywheel", tuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(RPM.of(50)));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Flywheel", m_inputs);

    boolean motor1HighCurrent = m_inputs.statorCurrent1.gt(Amps.of(68));
    boolean motor1NotMoving = Math.abs(m_inputs.velocity1.in(RotationsPerSecond)) < 2;
    boolean motor2HighCurrent = m_inputs.statorCurrent2.gt(Amps.of(68));
    boolean motor2NotMoving = Math.abs(m_inputs.velocity2.in(RotationsPerSecond)) < 2;

    m_talon1ConnectionAlert.set(!m_inputs.motor1Connected);
    m_talon2ConnectionAlert.set(!m_inputs.motor2Connected);
    m_stallAlert1.set(motor1HighCurrent && motor1NotMoving);
    m_stallAlert2.set(motor2HighCurrent && motor2NotMoving);

    Logger.recordOutput("Flywheel/isNearSetpoint", isNearSetpointTrigger().getAsBoolean());

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

  public Command setSpeedCommandWithFeedForward(Supplier<AngularVelocity> targetSpeedSupplier) {
    return run(() -> {
      AngularVelocity setpoint = targetSpeedSupplier.get();
			AngularVelocity currentSpeed = getSpeed();
      AngularVelocity error = setpoint.minus(currentSpeed);

			Voltage feedForwardVoltage = Volts.mutable(0);
			if (error.gt(RPM.of(200))) {
				double v = MathUtil.clamp(error.div(1000).in(RPM), .3, 1);
				feedForwardVoltage = Volts.of(v * 0.6);
			}
      m_io.setSpeed(setpoint, feedForwardVoltage);
    });
  }

  public Command slowIdle() {
    return run(() -> m_io.setSpeed(RPM.of(100)));
  }

  public Command stop() {
    return runOnce(() -> m_io.stop());
  }

  public Command tuningMode() {
    return setSpeedCommand(() -> RPM.of(m_shooterSetpointSpeed.getAsDouble()));
  }
}
