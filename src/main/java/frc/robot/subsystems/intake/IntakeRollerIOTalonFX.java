package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Ports;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private static final double kGearRatio = 1.0;

  private final TalonFX m_motor;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;

  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private Voltage m_setpoint = Volts.zero();

  public IntakeRollerIOTalonFX() {
    m_motor = new TalonFX(Ports.kIntakeRoller.canId(), Ports.kIntakeRoller.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio);

    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(90))
				.withSupplyCurrentLimitEnable(true)
				.withSupplyCurrentLimit(Amps.of(30));

    config.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(2.0));

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
  public void updateInputs(IntakeRollerIOInputs inputs) {
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
  public void setVoltage(Voltage voltage) {
    m_setpoint = voltage;
    m_motor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }
}
