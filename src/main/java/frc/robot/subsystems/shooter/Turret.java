package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Turret/kP", 20);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Turret/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Turret/kD", 0);
  private final LoggedTunableNumber m_turretSetpointAngleDegrees = new LoggedTunableNumber(
      "Turret/SetpointDegrees", 0);

  public Turret(TurretIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/Turret", tuningMode());
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(Degrees.of(1)));
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("Turret", m_inputs);

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(m_inputs.statorCurrent.gt(Amps.of(68)));

    LoggedTunableNumber.ifChanged(
        hashCode(), values -> m_io.setPID(values[0], values[1], values[2]), m_kP, m_kI, m_kD);

    Logger.recordOutput("Turret/angle", getAngle());

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

  public Command setAngleCommand(Supplier<Angle> angle) {
    Supplier<Angle> newAngle = mapSupplier(angle, this::findNearestAngle);
    return run(() -> m_io.setPosition(newAngle.get()));
  }

  public Command tuningMode() {
    return setAngleCommand(() -> Degrees.of(m_turretSetpointAngleDegrees.getAsDouble()));
  }

  private Angle findNearestAngle(Angle angle) {
    double targetDegrees = angle.in(Degrees);
    double currentDegrees = getAngle().in(Degrees);

    double delta = ((targetDegrees - currentDegrees) % 360 + 540) % 360 - 180;
    double bestAngle = currentDegrees + delta;

    if (bestAngle > 255.0) {
      bestAngle = bestAngle - 360.0;
    } else if (bestAngle < -255.0) {
      bestAngle = bestAngle + 360.0;
    }

    bestAngle = MathUtil.clamp(bestAngle, -255, 255);

    return Degrees.of(bestAngle);
  }

  private static <T> Supplier<T> mapSupplier(Supplier<T> supplier, Function<T, T> mapper) {
    return () -> mapper.apply(supplier.get());
  }
}
