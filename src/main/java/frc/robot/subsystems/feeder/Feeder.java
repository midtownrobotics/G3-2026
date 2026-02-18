package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final SmartMotorController m_feederMotor;
  private final FlyWheel m_feeder;
  private final CANrange m_fuelSensor;
  private final LinearFilter m_fuelSensorFilter;

  public Feeder() {
    SmartMotorControllerConfig beltMotorCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withClosedLoopController(0.3, 0, 0.01)
        .withFeedforward(new SimpleMotorFeedforward(0.05, 0.12, 0))
        .withGearing(4)
        .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    TalonFX beltTalonFX = new TalonFX(Ports.kFeederBeltTalonFXPort.canId(), Ports.kFeederBeltTalonFXPort.canbus());
    m_feederMotor = new TalonFXWrapper(beltTalonFX, DCMotor.getKrakenX44(1), beltMotorCfg);

    FlyWheelConfig beltConfig = new FlyWheelConfig(m_feederMotor)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withDiameter(Inches.of(2.0))
        .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

    m_feeder = new FlyWheel(beltConfig);

    CANrangeConfiguration fuelSensorConfig = new CANrangeConfiguration();

    m_fuelSensor = new CANrange(Ports.kFeederFuelSensor.canId(), Ports.kFeederFuelSensor.canbus());
    m_fuelSensor.getConfigurator().apply(fuelSensorConfig);

    m_fuelSensorFilter = LinearFilter.movingAverage(5);
  }

  private boolean getFuelSensorTripped() {
    return m_fuelSensorFilter.calculate(m_fuelSensor.getDistance().getValue().baseUnitMagnitude()) < Inches.of(5).baseUnitMagnitude();
  }

  public Trigger fuelSensorTripped() {
    return new Trigger(this::getFuelSensorTripped).debounce(Milliseconds.of(100).in(Seconds));
  }

  @Override
  public void periodic() {
    DogLog.log("Feeder/FuelSensor/Distance", m_fuelSensor.getDistance().getValue());
    DogLog.log("Feeder/FuelSensor/DistanceSTD", m_fuelSensor.getDistanceStdDev().getValue());
    m_feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feeder.simIterate();
  }

  public Command setSpeedCommand(AngularVelocity angularVelocity) {
    return m_feeder.setSpeed(angularVelocity);
  }
}
