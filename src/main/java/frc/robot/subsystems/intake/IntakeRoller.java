package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
public class IntakeRoller extends SubsystemBase {
  private final Alert m_talonConnectionAlert = new Alert("IntakeRoller TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("IntakeRoller stalling", AlertType.kWarning);
  private final TalonFX m_motor;
  private final Watchdawg m_watchdog;

  public IntakeRoller() {
    m_motor = new TalonFX(Ports.kIntakeRoller.canId(), Ports.kIntakeRoller.canbus());
    m_watchdog = new Watchdawg(this.getClass());
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    config.MotionMagic.withMotionMagicCruiseVelocity(Rotations.per(Minute).of(6000));
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120));
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    m_talonConnectionAlert.set(!m_motor.isAlive());
    boolean highCurrent = m_motor.getStatorCurrent().getValueAsDouble() > 110;
    boolean notMoving = Math.abs(m_motor.getVelocity().getValueAsDouble()) < 2;
    m_stallAlert.set(highCurrent && notMoving);
    m_watchdog.end("updateAlerts");
  }

  public Command setVoltageCommand(Voltage volts) {
    return Commands.runOnce(() -> m_motor.setVoltage(volts.in(Volts)));
  }
}