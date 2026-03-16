package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import frc.lib.Watchdawg;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO m_io;
  private final IntakePivotIOInputsAutoLogged m_inputs = new IntakePivotIOInputsAutoLogged();
  private final Alert m_talonConnectionAlert = new Alert("IntakePivot TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("IntakePivot stalling", AlertType.kWarning);
  private final Watchdawg m_watchdog;

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("IntakePivot/kP", 70.0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("IntakePivot/kI", 0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("IntakePivot/kD", 0);
  private final LoggedTunableNumber m_kG = new LoggedTunableNumber("IntakePivot/kG", 0.5);
  private final LoggedTunableNumber m_setpointAngle = new LoggedTunableNumber("IntakePivot/SetpointAngleDegrees", 0);

  public IntakePivot(IntakePivotIO io) {
    m_io = io;
    m_watchdog = new Watchdawg(getClass());
    SmartDashboard.putData("TuningModes/IntakePivot", tuningMode());
  }

  @Override
  public void periodic() {
    m_watchdog.start();

    m_io.updateInputs(m_inputs);
    Logger.processInputs("IntakePivot", m_inputs);

    boolean highCurrent = m_inputs.statorCurrent.gt(Amps.of(68));

    m_talonConnectionAlert.set(!m_inputs.motorConnected);
    m_stallAlert.set(highCurrent);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> m_io.setPID(values[0], values[1], values[2], values[3]),
        m_kP,
        m_kI,
        m_kD,
        m_kG);

    m_watchdog.end("periodic");
  }

  public Command setAngleCommand(Angle angle) {
    return run(() -> m_io.setPosition(angle));
  }

  public Command setAngleCommand(Supplier<Angle> angleSupplier) {
    return run(() -> m_io.setPosition(angleSupplier.get()));
  }

  public Command tuningMode() {
    return setAngleCommand(() -> Degrees.of(m_setpointAngle.getAsDouble()));
  }

  public Angle getAngle() {
    return m_inputs.position;
  }
}
