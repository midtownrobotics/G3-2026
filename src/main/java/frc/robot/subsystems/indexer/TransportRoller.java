package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class TransportRoller extends SubsystemBase {
  private final FlyWheel m_mechanism;
  private final Alert m_talonConnectionAlert = new Alert("TransportRoller TalonFX motor is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("TransportRoller stalling", AlertType.kWarning);
  private final TalonFX m_motor;

  public TransportRoller() {
    m_motor = new TalonFX(Ports.kIndexerTransportRoller.canId(), Ports.kIndexerTransportRoller.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("TransportRollerMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(60))
        .withGearing(20d / 14d);

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX60(1), motorControllerConfig);

    FlyWheelConfig rollerConfig = new FlyWheelConfig(motorController)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(5000))
        .withLowerSoftLimit(RPM.of(-5000))
        .withDiameter(Inches.of(1.5))
        .withTelemetry("TransportRoller", TelemetryVerbosity.LOW);

    m_mechanism = new FlyWheel(rollerConfig);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
    m_talonConnectionAlert.set(!m_motor.isAlive());
    boolean highCurrent = m_motor.getStatorCurrent().getValueAsDouble() > 45; 
    boolean notMoving = Math.abs(m_motor.getVelocity().getValueAsDouble()) < 2;
    m_stallAlert.set(highCurrent && notMoving);
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return m_mechanism.setSpeed(speed);
  }

  public Command setVoltageCommand(Voltage volts) {
    return m_mechanism.setVoltage(volts);
  }

  public Command stopCommand() {
    return m_mechanism.set(0.0);
  }
}
