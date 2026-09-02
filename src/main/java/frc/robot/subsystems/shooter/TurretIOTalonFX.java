package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Gains;
import frc.lib.MotionProfile;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class TurretIOTalonFX implements TurretIO {
  private static final double kRotorToSensorRatio = 60.0 / 12;
  private static final double kSensorToMechanismRatio = 82.0 / 10;

  private static final Angle kLowSoftLimit = Degrees.of(-90);
  private static final Angle kHighSoftLimit = Degrees.of(270);

  /**
   * Ceiling on closed-loop output. Under torque-current FOC this, not the stator current limit, is
   * what bounds the loop, so it is kept at the stator limit the mechanism was already validated at.
   */
  private static final Current kPeakTorqueCurrent = Amps.of(60);

  /**
   * Gains in amps, as tuned on the robot under torque-current FOC. These are already in output
   * units — they are not voltage gains and must not be rescaled again. kV is deliberately zero:
   * under torque control a zero output already holds a constant velocity, so kV is unnecessary.
   */
  public static final Gains kDefaultGains = new Gains(7000, 0, 80, 5, 0, 0, 0);

  public static final MotionProfile kDefaultMotionProfile = new MotionProfile(8, 9);

  private final TalonFX m_motor;
  private final CANcoder m_encoder1;
  // private final CANcoder m_encoder2;

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_torqueCurrentSignal;
  private final StatusSignal<Double> m_closedLoopReferenceSignal;
  private final StatusSignal<Double> m_closedLoopErrorSignal;
  private final StatusSignal<Angle> m_encoder1AbsolutePosition;
  // private final StatusSignal<Angle> m_encoder2AbsolutePosition;

  private final MotionMagicTorqueCurrentFOC m_positionRequest = new MotionMagicTorqueCurrentFOC(0);
  private final TorqueCurrentFOC m_torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private Angle m_setpoint = Degrees.zero();

  public TurretIOTalonFX() {
    m_motor = new TalonFX(Ports.kTurretYaw.canId(), Ports.kTurretYaw.canbus());
    m_encoder1 = new CANcoder(Ports.kTurretYawEncoder1.canId(), Ports.kTurretYawEncoder1.canbus());
    // m_encoder2 = new CANcoder(Ports.kTurretYawEncoder2.canId(), Ports.kTurretYawEncoder2.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = toSlot0(kDefaultGains);

    config.Feedback = new FeedbackConfigs()
        // .withRotorToSensorRatio(kRotorToSensorRatio)
        .withSensorToMechanismRatio(kSensorToMechanismRatio * kRotorToSensorRatio);
    // .withFusedCANcoder(m_encoder1);

    config.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    config.MotionMagic = toMotionMagic(kDefaultMotionProfile);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(60))
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(40));

    config.TorqueCurrent = new TorqueCurrentConfigs()
        .withPeakForwardTorqueCurrent(kPeakTorqueCurrent)
        .withPeakReverseTorqueCurrent(kPeakTorqueCurrent.unaryMinus());

    // The voltage ramps only bound the open-loop setVoltage path; torque requests need their own.
    config.OpenLoopRamps = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(Seconds.of(0.25))
        .withTorqueOpenLoopRampPeriod(Seconds.of(0.25));

    config.ClosedLoopRamps = new ClosedLoopRampsConfigs()
        .withTorqueClosedLoopRampPeriod(Seconds.of(0.05));

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(kHighSoftLimit)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(kLowSoftLimit);

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));

    // Configure CANcoders
    CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
    CANcoderConfiguration encoder2Config = new CANcoderConfiguration();

    encoder1Config.MagnetSensor.withMagnetOffset(Rotations.of(-0.18994140625))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    encoder2Config.MagnetSensor.withMagnetOffset(Rotations.of(0.21484375));

    PhoenixUtil.tryUntilOk(5, () -> m_encoder1.getConfigurator().apply(encoder1Config));
    // PhoenixUtil.tryUntilOk(5, () -> m_encoder2.getConfigurator().apply(encoder2Config));

    // Cache status signals
    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltsSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();
    m_torqueCurrentSignal = m_motor.getTorqueCurrent();
    m_closedLoopReferenceSignal = m_motor.getClosedLoopReference();
    m_closedLoopErrorSignal = m_motor.getClosedLoopError();
    m_encoder1AbsolutePosition = m_encoder1.getAbsolutePosition();
    // m_encoder2AbsolutePosition = m_encoder2.getAbsolutePosition();

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
        m_encoder1AbsolutePosition);
    // m_encoder2AbsolutePosition);

    // PhoenixUtil.tryUntilOk(5,
    // () -> BaseStatusSignal.refreshAll(m_encoder1AbsolutePosition, m_encoder2AbsolutePosition));

    // Seed motor position from CANcoder absolute position
    // EasyCRTConfig easyCRTConfig = new EasyCRTConfig(m_encoder1AbsolutePosition::getValue,
    //     m_encoder2AbsolutePosition::getValue)
    //     .withAbsoluteEncoder1GearingStages(82, 10)
    //     .withAbsoluteEncoder2GearingStages(82, 10, 20, 21)
    //     .withMechanismRange(kLowSoftLimit, kHighSoftLimit)
    //     .withMatchTolerance(Rotations.of(0.01));

    // EasyCRT easyCRT = new EasyCRT(easyCRTConfig);

    // easyCRT.getAngleOptional().ifPresentOrElse(m_motor::setPosition,
    //     () -> {
    //       DriverStation.reportError(
    //           "Unable to seed turret position from CANcoder absolute position\n" + "Encoder 1 position: "
    //               + m_encoder1AbsolutePosition.getValue().in(Degrees)
    //               + "\nEncoder 2 position: " + m_encoder2AbsolutePosition.getValue().in(Degrees),
    //           true);
    //       m_motor.setPosition(Degrees.of(90));
    //     });
    m_motor.setPosition(Degrees.of(90));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
        m_torqueCurrentSignal,
        m_closedLoopReferenceSignal,
        m_closedLoopErrorSignal,
        m_encoder1AbsolutePosition);
    // m_encoder2AbsolutePosition);

    inputs.position = m_positionSignal.getValue();
    inputs.velocity = m_velocitySignal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent = m_statorCurrentSignal.getValue();
    inputs.supplyCurrent = m_supplyCurrentSignal.getValue();
    inputs.torqueCurrent = m_torqueCurrentSignal.getValue();
    inputs.closedLoopReference = Rotations.of(m_closedLoopReferenceSignal.getValue());
    inputs.closedLoopError = Rotations.of(m_closedLoopErrorSignal.getValue());
    inputs.encoder1AbsolutePosition = m_encoder1AbsolutePosition.getValue();
    // inputs.encoder2AbsolutePosition = m_encoder2AbsolutePosition.getValue();
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = m_motor.isAlive();
  }

  @Override
  public void setPosition(Angle angle) {
    m_setpoint = mapAngleToLimits(angle);
    m_motor.setControl(m_positionRequest.withPosition(m_setpoint));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor.setControl(m_voltageRequest.withOutput(voltage));
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
  }

  /**
   * Maps an angle from any range to the range defined by the turret's soft limits.
   * @param angle
   * @return
   */
  private Angle mapAngleToLimits(Angle angle) {
    // Shift by +90 so that soft-limit low (-90) maps to 0, wrap to [0,360), then shift back.
    double mappedDegrees = (angle.in(Degrees) - kLowSoftLimit.in(Degrees) + 360) % 360
        + kLowSoftLimit.in(Degrees);
    return Degrees.of(mappedDegrees);
  }

  private static Slot0Configs toSlot0(Gains gains) {
    return new Slot0Configs()
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD())
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA())
        // Motion Magic always has a velocity setpoint to take the sign of, which avoids the
        // dithering that closed-loop-sign kS can cause when sitting on target.
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
  }

  /** Constraints are in mechanism rotations, matching the configured sensor-to-mechanism ratio. */
  private static MotionMagicConfigs toMotionMagic(MotionProfile profile) {
    return new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(profile.cruiseVelocity())
        .withMotionMagicAcceleration(profile.acceleration())
        .withMotionMagicJerk(profile.jerk());
  }

  @Override
  public void setGains(Gains gains) {
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(toSlot0(gains)));
  }

  @Override
  public void setMotionProfile(MotionProfile profile) {
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(toMotionMagic(profile)));
  }
}
