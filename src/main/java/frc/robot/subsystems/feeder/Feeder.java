package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggerUtil;
import frc.robot.Ports;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class Feeder extends SubsystemBase {
  private final FlyWheel m_mechanism;
  private final CANrange m_fuelSensor;
  private final LinearFilter m_fuelSensorFilter;
  private final Trigger m_fuelDetectedTrigger;

  public Feeder() {
    TalonFX motor = new TalonFX(Ports.kFeederBelt.canId(), Ports.kFeederBelt.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withClosedLoopController(0.3, 0, 0.01)
        .withFeedforward(new SimpleMotorFeedforward(0.05, 0.12, 0))
        .withGearing(2)
        .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), motorControllerConfig);

    FlyWheelConfig beltConfig = new FlyWheelConfig(motorController)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withDiameter(Inches.of(2.0))
        .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

    m_mechanism = new FlyWheel(beltConfig);

    m_fuelSensor = new CANrange(Ports.kFeederFuelSensor.canId(), Ports.kFeederFuelSensor.canbus());
    CANrangeConfiguration fuelSensorConfig = new CANrangeConfiguration();
    m_fuelSensor.getConfigurator().apply(fuelSensorConfig);

    m_fuelSensorFilter = LinearFilter.movingAverage(5);

    m_fuelDetectedTrigger = new Trigger(this::getFuelSensorTripped).debounce(Milliseconds.of(100).in(Seconds));
  }

  private boolean getFuelSensorTripped() {
    return m_fuelSensorFilter.calculate(m_fuelSensor.getDistance().getValue().baseUnitMagnitude()) < Inches.of(5)
        .baseUnitMagnitude();
  }

  public Trigger fuelDetected() {
    return m_fuelDetectedTrigger;
  }

  @Override
  public void periodic() {
    LoggerUtil.log("FuelSensor/distance", m_fuelSensor.getDistance().getValue());
    LoggerUtil.log("FuelSensor/distanceSTD", m_fuelSensor.getDistanceStdDev().getValue());
    LoggerUtil.log("sensorTripped", getFuelSensorTripped());
    m_mechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Command setSpeedCommand(AngularVelocity angularVelocity) {
    return m_mechanism.setSpeed(angularVelocity);
  }
}
