package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;

public class IndexerIOTalonFX implements InexerIO {
  private static final double kGearRatio = 20.0 / 14.0;

  private final TalonFX m_motor;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private AngularVelocity m_setpoint = RPM.zero();

  public IndexerIOTalonFX() {
    m_motor = new TalonFX(Ports.kIndexer.canId(), Ports.kIndexer.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);

    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio);

    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(60))
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(10));

    m_motor.getConfigurator().apply(config);

    // Cache status signals
    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltsSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, m_velocitySignal, m_appliedVoltsSignal, m_statorCurrentSignal, m_supplyCurrentSignal);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_velocitySignal, m_appliedVoltsSignal, m_statorCurrentSignal, m_supplyCurrentSignal);

    inputs.velocity = m_velocitySignal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent = m_statorCurrentSignal.getValue();
    inputs.supplyCurrent = m_supplyCurrentSignal.getValue();
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = m_motor.isAlive();
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    m_setpoint = speed;
    m_motor.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
    m_motor.getConfigurator().apply(slot0);
  }
}
