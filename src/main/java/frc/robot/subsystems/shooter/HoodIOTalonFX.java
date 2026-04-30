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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

public class HoodIOTalonFX implements HoodIO {
  private static final double kSensorToMechanismRatio = 19.0;
  private static final double kRotorToSensorRatio = 14.0;
  private static final Angle kMagnetOffset = Degrees.of(0).times(kSensorToMechanismRatio);

  private final TalonFX m_motor;
  private final CANcoder m_encoder;

  private final StatusSignal<Angle> m_positionSignal;
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> m_appliedVoltsSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_statorCurrentSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> m_supplyCurrentSignal;
  private final StatusSignal<Angle> m_encoderAbsolutePosition;

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withEnableFOC(true);
	
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private Angle m_setpoint = Degrees.zero();

  public HoodIOTalonFX() {
    m_motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
    m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = new Slot0Configs()
        .withKP(800)
        .withKI(0)
        .withKD(70)
        .withKS(0)
        .withKV(0)
        .withKG(0)
        .withGravityArmPositionOffset(Degrees.of(11))
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.Feedback = new FeedbackConfigs()
        .withSensorToMechanismRatio(kSensorToMechanismRatio)
        .withRotorToSensorRatio(kRotorToSensorRatio)
        .withFusedCANcoder(m_encoder);

    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(40));

    config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Degrees.of(40))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Degrees.of(0));

    config.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(0.25));

    config.ClosedLoopRamps = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(Seconds.of(0.25));

    PhoenixUtil.tryUntilOk(5,  () -> m_motor.getConfigurator().apply(config));

    // Cache status signals
    m_positionSignal = m_motor.getPosition();
    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltsSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();
    m_encoderAbsolutePosition = m_encoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_positionSignal,
        m_velocitySignal,
        m_appliedVoltsSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal,
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
        m_encoderAbsolutePosition);

    inputs.position = m_positionSignal.getValue();
    inputs.velocity = m_velocitySignal.getValue();
    inputs.appliedVoltage = m_appliedVoltsSignal.getValue();
    inputs.statorCurrent = m_statorCurrentSignal.getValue();
    inputs.supplyCurrent = m_supplyCurrentSignal.getValue();
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

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kG) {
    var slot0 = new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKG(kG)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    m_motor.getConfigurator().apply(slot0);
  }
}
