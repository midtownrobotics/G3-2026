package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double kGearRatio = 1.0;

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity2Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrent1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrent2Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrent1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrent2Signal;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private AngularVelocity m_setpoint = RPM.zero();
  private Voltage m_feedForward = Volts.zero();

  public FlywheelIOTalonFX() {
    m_motor1 = new TalonFX(Ports.kTurretShooter1.canId(), Ports.kTurretShooter1.canbus());
    m_motor2 = new TalonFX(Ports.kTurretShooter2.canId(), Ports.kTurretShooter2.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = new Slot0Configs()
        .withKP(0.38)
        .withKI(0)
        .withKD(0)
        .withKS(0.29)
        .withKV(0.13)
        .withKA(0);
    config.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback
        .withSensorToMechanismRatio(kGearRatio);
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120));
    PhoenixUtil.tryUntilOk(5, () -> m_motor1.getConfigurator().apply(config));

    // Motor 2 follows motor 1 inverted
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    // Cache status signals
    m_velocity1Signal = m_motor1.getVelocity();
    m_velocity2Signal = m_motor2.getVelocity();
    m_appliedVoltsSignal = m_motor1.getMotorVoltage();
    m_statorCurrent1Signal = m_motor1.getStatorCurrent();
    m_statorCurrent2Signal = m_motor2.getStatorCurrent();
    m_supplyCurrent1Signal = m_motor1.getSupplyCurrent();
    m_supplyCurrent2Signal = m_motor2.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_velocity1Signal,
        m_velocity2Signal,
        m_appliedVoltsSignal,
        m_statorCurrent1Signal,
        m_statorCurrent2Signal,
        m_supplyCurrent1Signal,
        m_supplyCurrent2Signal);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_velocity1Signal,
        m_velocity2Signal,
        m_appliedVoltsSignal,
        m_statorCurrent1Signal,
        m_statorCurrent2Signal,
        m_supplyCurrent1Signal,
        m_supplyCurrent2Signal);

    inputs.velocity = m_velocity1Signal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent1 = m_statorCurrent1Signal.getValue();
    inputs.statorCurrent2 = m_statorCurrent2Signal.getValue();
    inputs.supplyCurrent1 = m_supplyCurrent1Signal.getValue();
    inputs.supplyCurrent2 = m_supplyCurrent2Signal.getValue();
    inputs.velocity1 = m_velocity1Signal.getValue();
    inputs.velocity2 = m_velocity2Signal.getValue();
    inputs.setpoint = m_setpoint;
    inputs.feedForwardVoltage = m_feedForward;
    inputs.motor1Connected = m_motor1.isAlive();
    inputs.motor2Connected = m_motor2.isAlive();
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    m_setpoint = speed;
    m_motor1.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  @Override
  public void setSpeed(AngularVelocity speed, Voltage feedForward) {
    m_setpoint = speed;
    m_feedForward = feedForward;
    m_motor1.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)).withFeedForward(feedForward));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor1.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void bangBang(Voltage voltage, AngularVelocity targetSpeed) {
    m_setpoint = targetSpeed;
    setVoltage(voltage);
  }

  @Override
  public void stop() {
    m_setpoint = RPM.of(0);
    m_motor1.stopMotor();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV) {
    var slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV);
    m_motor1.getConfigurator().apply(slot0);
  }
}
