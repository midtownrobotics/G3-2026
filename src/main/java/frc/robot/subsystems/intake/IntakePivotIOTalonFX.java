package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  // (50/12) * (60/20) * (48/16) = 37.5
  private static final double kRotorToSensorRatio = (50.0 / 12.0) * (60.0 / 20.0);
  private static final double kSensorToMechanismRatio = 48.0 / 16.0;

  private final TalonFX m_motor;
  private final CANcoder m_encoder;

  private final StatusSignal<edu.wpi.first.units.measure.Angle> m_positionSignal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Angle> m_encoderPositionSignal;

  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private Angle m_setpoint = Degrees.zero();

  public IntakePivotIOTalonFX() {
    m_motor = new TalonFX(Ports.kIntakePivot.canId(), Ports.kIntakePivot.canbus());
    m_encoder = new CANcoder(Ports.kIntakePivotEncoder.canId(), Ports.kIntakePivotEncoder.canbus());

    // Configure CANcoder
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withAbsoluteSensorDiscontinuityPoint(Degrees.of(350))
        .withMagnetOffset(Rotations.of(0.123779296875));

    PhoenixUtil.tryUntilOk(5, () -> m_encoder.getConfigurator().apply(canCoderConfig));

    // Configure TalonFX
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = new Slot0Configs()
        .withKP(70.0)
        .withKI(0)
        .withKD(0)
        .withKG(0.5)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.Feedback = new FeedbackConfigs()
        .withRotorToSensorRatio(kRotorToSensorRatio)
        .withSensorToMechanismRatio(kSensorToMechanismRatio)
        .withFusedCANcoder(m_encoder);

    config.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(50));

    config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(DegreesPerSecond.of(540))
        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(1200))
        .withMotionMagicJerk(10);

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(70))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Degrees.of(0));

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));

    // Cache status signals
    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltsSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();
    m_encoderPositionSignal = m_encoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
        m_encoderPositionSignal);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
        m_encoderPositionSignal);

    inputs.position = m_positionSignal.getValue();
    inputs.velocity = m_velocitySignal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent = m_statorCurrentSignal.getValue();
    inputs.supplyCurrent = m_supplyCurrentSignal.getValue();
    inputs.encoderAbsolutePosition = m_encoderPositionSignal.getValue();
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = m_motor.isAlive();
  }

  @Override
  public void setPosition(Angle angle) {
    m_setpoint = angle;
    m_motor.setControl(m_motionMagicRequest.withPosition(angle));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_motor.setControl(m_voltageRequest.withOutput(voltage));
  }

  @Override
  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    m_motor.setPosition(angle);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kG) {
    var slot0 = new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKG(kG)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    m_motor.getConfigurator().apply(slot0);
  }

  @Override
  public void setLowerSoftLimitEnabled(boolean enabled) {
    SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(70))
        .withReverseSoftLimitEnable(enabled)
        .withReverseSoftLimitThreshold(Degrees.of(0));

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }
}
