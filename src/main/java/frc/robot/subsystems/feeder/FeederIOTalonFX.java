package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class FeederIOTalonFX implements FeederIO {
  private static final double kGearRatio = 2.0;

  private final TalonFX m_motorLeader;
  private final TalonFX m_motorFollower;
  // private final CANrange m_fuelSensor;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal1;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal1;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal1;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal1;
	  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal2;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal2;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal2;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal2;
  // private final StatusSignal<Distance> m_fuelSensorDistanceSignal;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private AngularVelocity m_setpoint = RPM.zero();

  public FeederIOTalonFX() {
    m_motorLeader = new TalonFX(Ports.kFeederBeltLeader.canId(), Ports.kFeederBeltLeader.canbus());
    m_motorFollower = new TalonFX(Ports.kFeederBeltFollower.canId(), Ports.kFeederBeltFollower.canbus());
    // m_fuelSensor = new CANrange(Ports.kFeederFuelSensor.canId(), Ports.kFeederFuelSensor.canbus());

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio);

    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(40));

    PhoenixUtil.tryUntilOk(5, () -> m_motorLeader.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> m_motorFollower.getConfigurator().apply(config));

    // Configure CANrange
    CANrangeConfiguration fuelSensorConfig = new CANrangeConfiguration();
    // m_fuelSensor.getConfigurator().apply(fuelSensorConfig);

    // Cache status signals
    m_velocitySignal1 = m_motorLeader.getVelocity();
    m_appliedVoltsSignal1 = m_motorLeader.getMotorVoltage();
    m_statorCurrentSignal1 = m_motorLeader.getStatorCurrent();
    m_supplyCurrentSignal1 = m_motorLeader.getSupplyCurrent();

		m_velocitySignal2 = m_motorFollower.getVelocity();
    m_appliedVoltsSignal2 = m_motorFollower.getMotorVoltage();
    m_statorCurrentSignal2 = m_motorFollower.getStatorCurrent();
    m_supplyCurrentSignal2 = m_motorFollower.getSupplyCurrent();
    // m_fuelSensorDistanceSignal = m_fuelSensor.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_velocitySignal1,
        m_appliedVoltsSignal1,
        m_statorCurrentSignal1,
        m_supplyCurrentSignal1,
				m_velocitySignal2,
        m_appliedVoltsSignal2,
        m_statorCurrentSignal2,
        m_supplyCurrentSignal2);
    // m_fuelSensorDistanceSignal);

    m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_velocitySignal1,
        m_appliedVoltsSignal1,
        m_statorCurrentSignal1,
        m_supplyCurrentSignal1,
				m_velocitySignal2,
        m_appliedVoltsSignal2,
        m_statorCurrentSignal2,
        m_supplyCurrentSignal2);
    // m_fuelSensorDistanceSignal);

		inputs.appliedVoltage1 = m_appliedVoltsSignal1.getValue();
		inputs.appliedVoltage2 = m_appliedVoltsSignal2.getValue();
    inputs.velocity1 = m_velocitySignal1.getValue();
		inputs.velocity2 = m_velocitySignal2.getValue();
    inputs.statorCurrent1 = m_statorCurrentSignal1.getValue();
		inputs.statorCurrent2 = m_statorCurrentSignal2.getValue();
    inputs.supplyCurrent1 = m_supplyCurrentSignal1.getValue();
		inputs.supplyCurrent2 = m_supplyCurrentSignal2.getValue();
    // inputs.fuelSensorDistance = m_fuelSensorDistanceSignal.getValue();
    inputs.setpoint = m_setpoint;
    inputs.motorConnected1 = m_motorLeader.isAlive();
		inputs.motorConnected2 = m_motorFollower.isAlive();
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    m_setpoint = speed;
    m_motorLeader.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motorLeader.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stop() {
    m_motorLeader.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
    m_motorLeader.getConfigurator().apply(slot0);
  }
}
