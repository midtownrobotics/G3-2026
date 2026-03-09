package frc.robot.subsystems.feeder;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Logger;
import frc.lib.PhoenixUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;

@Logged(strategy = Strategy.OPT_IN)
public class Feeder extends SubsystemBase {
  private final CANrange m_fuelSensor;
  private final LinearFilter m_fuelSensorFilter;
  private final Logger m_log;
  private final Alert m_talonConnectionAlert = new Alert("Feeder TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Feeder stalling", AlertType.kWarning);
  private final TalonFX m_motor;
  private final Watchdawg m_watchdog;

  public Feeder() {
    m_motor = new TalonFX(Ports.kFeederBelt.canId(), Ports.kFeederBelt.canbus());
    m_fuelSensor = new CANrange(Ports.kFeederFuelSensor.canId(), Ports.kFeederFuelSensor.canbus());
    CANrangeConfiguration fuelSensorConfig = new CANrangeConfiguration();
    m_fuelSensor.getConfigurator().apply(fuelSensorConfig);

    m_fuelSensorFilter = LinearFilter.movingAverage(5);
    m_log = new Logger(getClass());
    m_watchdog = new Watchdawg(getClass());

    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(90);

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
    
  }

  private boolean getFuelSensorTripped() {
    return m_fuelSensorFilter.calculate(m_fuelSensor.getDistance().getValue().baseUnitMagnitude()) < Inches.of(5)
        .baseUnitMagnitude();
  }

  public Trigger fuelSensorTripped() {
    return new Trigger(this::getFuelSensorTripped).debounce(Milliseconds.of(100).in(Seconds));
  }

  @Override
  public void periodic() {
    m_log.log("FuelSensor/distance", m_fuelSensor.getDistance().getValue());
    m_log.log("FuelSensor/distanceSTD", m_fuelSensor.getDistanceStdDev().getValue());
    m_log.log("sensorTripped", getFuelSensorTripped());

    m_watchdog.start();
    m_talonConnectionAlert.set(!m_motor.isAlive());
    boolean highCurrent = m_motor.getStatorCurrent().getValueAsDouble() > 30;
    boolean notMoving = Math.abs(m_motor.getVelocity().getValueAsDouble()) < 2;
    m_stallAlert.set(highCurrent && notMoving);
    m_watchdog.end("updateAlerts");
  }

  public Command setVoltageCommand(Voltage volts) {
    return Commands.runOnce(() -> m_motor.setVoltage(volts.baseUnitMagnitude()));
  }

}
