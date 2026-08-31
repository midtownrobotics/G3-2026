package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Gains;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class HoodIOTalonFX implements HoodIO {
  private static final double kSensorToMechanismRatio = 19.0;
  private static final double kRotorToSensorRatio = 14.0;
  private static final Angle kMagnetOffset = Degrees.of(0).times(kSensorToMechanismRatio);

  /** Offset of the arm's center of mass from horizontal, used to orient the kG cosine term. */
  private static final Angle kGravityArmPositionOffset = Degrees.of(11);

  /**
   * Ceiling on closed-loop output. Under torque-current FOC this, not the stator current limit, is
   * what bounds the loop, so it is kept at the stator limit the mechanism was already validated at.
   */
  private static final Current kPeakTorqueCurrent = Amps.of(40);

  /**
   * Starting gains, in amps, converted from the voltage gains this hood was tuned with
   * (kP 800, kD 70). Verify these on the robot — the conversion is a first-order estimate.
   */
  public static final Gains kDefaultGains =
      Gains.fromKrakenVoltageGains(new Gains(0, 0, 0, 0, 0, 0, 0));

  private final TalonFX m_motor;
  private final CANcoder m_encoder;

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_torqueCurrentSignal;
  private final StatusSignal<Double> m_closedLoopReferenceSignal;
  private final StatusSignal<Double> m_closedLoopErrorSignal;
  private final StatusSignal<Angle> m_encoderAbsolutePosition;

  private final PositionTorqueCurrentFOC m_positionRequest = new PositionTorqueCurrentFOC(0);
  private final TorqueCurrentFOC m_torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private Angle m_setpoint = Degrees.zero();

  public HoodIOTalonFX() {
    m_motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
    m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = toSlot0(kDefaultGains);

    config.Feedback = new FeedbackConfigs()
        .withSensorToMechanismRatio(kSensorToMechanismRatio)
        .withRotorToSensorRatio(kRotorToSensorRatio)
        .withFusedCANcoder(m_encoder);

    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(40));

    config.TorqueCurrent = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(kPeakTorqueCurrent)
        .withPeakReverseTorqueCurrent(kPeakTorqueCurrent.unaryMinus());

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(40))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Degrees.of(0));

    // The voltage ramps only bound the open-loop setVoltage path; torque requests need their own.
    config.OpenLoopRamps = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(Seconds.of(0.25))
        .withTorqueOpenLoopRampPeriod(Seconds.of(0.25));

    config.ClosedLoopRamps = new ClosedLoopRampsConfigs()
        .withTorqueClosedLoopRampPeriod(Seconds.of(0.05));

    PhoenixUtil.tryUntilOk(5,  () -> m_motor.getConfigurator().apply(config));

    // Cache status signals
    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltsSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();
    m_torqueCurrentSignal = m_motor.getTorqueCurrent();
    m_closedLoopReferenceSignal = m_motor.getClosedLoopReference();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    m_encoderAbsolutePosition = m_encoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
        m_torqueCurrentSignal,
        m_closedLoopReferenceSignal,
        m_closedLoopErrorSignal,
        m_encoderAbsolutePosition);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
        m_torqueCurrentSignal,
        m_closedLoopReferenceSignal,
        m_closedLoopErrorSignal,
        m_encoderAbsolutePosition);

    inputs.position = m_positionSignal.getValue();
    inputs.velocity = m_velocitySignal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent = m_statorCurrentSignal.getValue();
    inputs.supplyCurrent = m_supplyCurrentSignal.getValue();
    inputs.torqueCurrent = m_torqueCurrentSignal.getValue();
    inputs.closedLoopReference = Rotations.of(m_closedLoopReferenceSignal.getValue());
    inputs.closedLoopError = Rotations.of(m_closedLoopErrorSignal.getValue());
    inputs.encoderAbsolutePosition = m_encoderAbsolutePosition.getValue();
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = m_motor.isAlive();
  }

  @Override
  public void setPosition(Angle angle) {
    m_setpoint = angle;
    m_motor.setControl(m_positionRequest.withPosition(angle.in(Rotations)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  @Override
  public void setTorqueCurrent(Current current) {
    m_motor.setControl(m_torqueCurrentRequest.withOutput(current));
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    m_motor.setPosition(angle);
    m_encoder.setPosition(angle);
  }

  @Override
  public void setLowerSoftLimitEnabled(boolean enabled) {
    SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(40))
        .withReverseSoftLimitEnable(enabled)
        .withReverseSoftLimitThreshold(Degrees.of(0));

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }

  private static Slot0Configs toSlot0(Gains gains) {
    return new Slot0Configs()
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD())
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA())
        .withKG(gains.kG())
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withGravityArmPositionOffset(kGravityArmPositionOffset)
        // This is an unprofiled position loop, so there is no velocity setpoint for kS to take its
        // sign from; derive it from the closed-loop error instead. Keep kS small or the hood will
        // dither once it reaches the target.
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  }

  @Override
  public void setGains(Gains gains) {
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(toSlot0(gains)));
  }
}
