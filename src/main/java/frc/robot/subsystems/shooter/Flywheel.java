package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import frc.lib.TunableGains;
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
  private final LoggedTunableNumber m_openLoopTorqueAmps = new LoggedTunableNumber(
      "Flywheel/OpenLoopTorqueAmps", 0);

  // Extra current injected while spinning up, on top of the velocity loop. Defaults reproduce the
  // voltage-based spin-up boost this shooter previously used, converted into amps.
  private final LoggedTunableNumber m_spinUpThresholdRPM = new LoggedTunableNumber(
      "Flywheel/SpinUp/ThresholdRPM", 200);
  private final LoggedTunableNumber m_spinUpAmpsPerRPM = new LoggedTunableNumber(
      "Flywheel/SpinUp/AmpsPerRPM", 0.02415);
  private final LoggedTunableNumber m_spinUpMinAmps = new LoggedTunableNumber(
      "Flywheel/SpinUp/MinAmps", 7.25);
  private final LoggedTunableNumber m_spinUpMaxAmps = new LoggedTunableNumber(
      "Flywheel/SpinUp/MaxAmps", 24.15);

  private final TunableGains m_gains = new TunableGains("Flywheel", FlywheelIOTalonFX.kDefaultGains);

  public Flywheel(FlywheelIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Flywheel", tuningMode());
    SmartDashboard.putData("TuningModes/FlywheelOpenLoopTorque", openLoopTorqueTuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(RPM.of(50)));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Flywheel", m_inputs);

    m_gains.poll(hashCode(), m_io::setGains);

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

      Current feedForward = Amps.zero();
      if (error.gt(RPM.of(m_spinUpThresholdRPM.get()))) {
        feedForward = Amps.of(MathUtil.clamp(
            error.in(RPM) * m_spinUpAmpsPerRPM.get(),
            m_spinUpMinAmps.get(),
            m_spinUpMaxAmps.get()));
      }
      m_io.setSpeed(setpoint, feedForward);
    });
  }

  public Command slowIdle() {
    return run(() -> m_io.setSpeed(RPM.of(100)));
  }

  public Command stop() {
    return runOnce(() -> m_io.stop());
  }

  /** Spins to {@code /Tuning/Flywheel/SetpointRPM} so the velocity loop can be tuned live. */
  public Command tuningMode() {
    return setSpeedCommand(() -> RPM.of(m_shooterSetpointSpeed.getAsDouble()))
        .withName("flywheelTuningMode");
  }

  /**
   * Commands raw torque current from {@code /Tuning/Flywheel/OpenLoopTorqueAmps}, bypassing the
   * velocity loop. The current that just barely keeps the wheel turning is kS; the steady-state
   * speed reached at a fixed current characterizes the wheel's drag.
   */
  public Command openLoopTorqueTuningMode() {
    return run(() -> m_io.setTorqueCurrent(Amps.of(m_openLoopTorqueAmps.getAsDouble())))
        .finallyDo(m_io::stop)
        .withName("flywheelOpenLoopTorqueTuning");
  }
}
