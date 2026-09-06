package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Gains;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double kGearRatio = 1.0;

  /**
   * Ceiling on closed-loop output. Under torque-current FOC this, not the stator current limit, is
   * what bounds the loop, so it is kept at the stator limit the mechanism was already validated at.
   */
  private static final Current kPeakTorqueCurrent = Amps.of(120);

  /**
   * Gains in amps, tuned on the robot under torque-current FOC — not voltage gains, so nothing
   * rescales them. kV is deliberately zero: under torque control a zero output already holds a
   * constant velocity. If the wheel settles below its target under load, add kA or a little kI
   * rather than reintroducing kV.
   */
  public static final Gains kDefaultGains = new Gains(10, 0, 0, 11.75, 0.2, 0, 0);

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity2Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrent1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrent2Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrent1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrent2Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_torqueCurrent1Signal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_torqueCurrent2Signal;
  private final StatusSignal<Double> m_closedLoopReferenceSignal;
  private final StatusSignal<Double> m_closedLoopErrorSignal;

  private final VelocityTorqueCurrentFOC m_velocityRequest = new VelocityTorqueCurrentFOC(0);
  private final TorqueCurrentFOC m_torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private AngularVelocity m_setpoint = RPM.zero();
  private Current m_feedForward = Amps.zero();

  public FlywheelIOTalonFX() {
    m_motor1 = new TalonFX(Ports.kTurretShooter1.canId(), Ports.kTurretShooter1.canbus());
    m_motor2 = new TalonFX(Ports.kTurretShooter2.canId(), Ports.kTurretShooter2.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = toSlot0(kDefaultGains);
    config.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback
        .withSensorToMechanismRatio(kGearRatio);
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120));
    config.TorqueCurrent = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(kPeakTorqueCurrent)
        .withPeakReverseTorqueCurrent(kPeakTorqueCurrent.unaryMinus());
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
    m_torqueCurrent1Signal = m_motor1.getTorqueCurrent();
    m_torqueCurrent2Signal = m_motor2.getTorqueCurrent();
    m_closedLoopReferenceSignal = m_motor1.getClosedLoopReference();
    m_closedLoopErrorSignal = m_motor1.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_velocity1Signal,
        m_velocity2Signal,
        m_appliedVoltsSignal,
        m_statorCurrent1Signal,
        m_statorCurrent2Signal,
        m_supplyCurrent1Signal,
        m_supplyCurrent2Signal,
        m_torqueCurrent1Signal,
        m_torqueCurrent2Signal,
        m_closedLoopReferenceSignal,
        m_closedLoopErrorSignal);
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
        m_supplyCurrent2Signal,
        m_torqueCurrent1Signal,
        m_torqueCurrent2Signal,
        m_closedLoopReferenceSignal,
        m_closedLoopErrorSignal);

    inputs.velocity = m_velocity1Signal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent1 = m_statorCurrent1Signal.getValue();
    inputs.statorCurrent2 = m_statorCurrent2Signal.getValue();
    inputs.supplyCurrent1 = m_supplyCurrent1Signal.getValue();
    inputs.supplyCurrent2 = m_supplyCurrent2Signal.getValue();
    inputs.torqueCurrent1 = m_torqueCurrent1Signal.getValue();
    inputs.torqueCurrent2 = m_torqueCurrent2Signal.getValue();
    inputs.velocity1 = m_velocity1Signal.getValue();
    inputs.velocity2 = m_velocity2Signal.getValue();
    inputs.setpoint = m_setpoint;
    inputs.closedLoopReference = RotationsPerSecond.of(m_closedLoopReferenceSignal.getValue());
    inputs.closedLoopError = RotationsPerSecond.of(m_closedLoopErrorSignal.getValue());
    inputs.feedForwardCurrent = m_feedForward;
    inputs.motor1Connected = m_motor1.isAlive();
    inputs.motor2Connected = m_motor2.isAlive();
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    m_setpoint = speed;
    m_motor1.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)));
  }

  @Override
  public void setSpeed(AngularVelocity speed, Current feedForward) {
    m_setpoint = speed;
    m_feedForward = feedForward;
    m_motor1.setControl(m_velocityRequest.withVelocity(speed.in(RotationsPerSecond)).withFeedForward(feedForward));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor1.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void setTorqueCurrent(Current current) {
    m_motor1.setControl(m_torqueCurrentRequest.withOutput(current));
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

  private static Slot0Configs toSlot0(Gains gains) {
    return new Slot0Configs()
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD())
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA())
        // A velocity loop always has a velocity setpoint for kS to take its sign from.
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
  }

  @Override
  public void setGains(Gains gains) {
    PhoenixUtil.tryUntilOk(5, () -> m_motor1.getConfigurator().apply(toSlot0(gains)));
  }
}
