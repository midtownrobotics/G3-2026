package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PhoenixUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;

@Logged(strategy = Strategy.OPT_IN)
public class Indexer extends SubsystemBase {
  private final Alert m_talonConnectionAlert = new Alert("TransportRoller TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("TransportRoller stalling", AlertType.kWarning);
  private final TalonFX m_motor;
  private final Watchdawg m_watchdog;

  public Indexer() {
    m_motor = new TalonFX(Ports.kIndexerTransportRoller.canId(), Ports.kIndexerTransportRoller.canbus());
    m_watchdog = new Watchdawg(getClass());
    conifgureMotor();
  }

  private void conifgureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(90);

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    m_talonConnectionAlert.set(!m_motor.isAlive());
    boolean highCurrent = m_motor.getStatorCurrent().getValueAsDouble() > 68;
    boolean notMoving = Math.abs(m_motor.getVelocity().getValueAsDouble()) < 2;
    m_stallAlert.set(highCurrent && notMoving);
    m_watchdog.end("updateAlerts");
  }

  public Command setVoltageCommand(Voltage volts) {
    return Commands.runOnce(() -> m_motor.setVoltage(volts.baseUnitMagnitude()));
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> m_motor.setVoltage(0));
  }
}