package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Logger;
import frc.lib.PhoenixUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;

@Logged(strategy = Strategy.OPT_IN)
public class IntakePivot extends SubsystemBase {
  private static final double kMechanismToMotorGearing = 37.5;
  private static final double kMechanismToEncoderGearing = 3d;
  private static final Angle kLowSoftLimit = Degrees.of(0);
  private static final Angle kHighSoftLimit = Degrees.of(60);

  private final CANcoder m_encoder;
  private final TalonFX m_motor;
  private final Logger m_log;
  private final Watchdawg m_watchdog;
  private final Alert m_talonConnectionAlert = new Alert("IntakePivot motor controller is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("IntakePivot stalling", AlertType.kWarning);

  private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

  public IntakePivot() {
    m_motor = new TalonFX(Ports.kIntakePivot.canId(), Ports.kIntakePivot.canbus());
    m_encoder = new CANcoder(Ports.kIntakePivotEncoder.canId(), Ports.kIntakePivotEncoder.canbus());
    m_log = new Logger(getClass());
    m_watchdog = new Watchdawg(getClass());
    configureMotor();
    seedEncoderPosition();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = new Slot0Configs()
        .withKP(70)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKG(0.5)
        .withKV(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    config.MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.MotionMagic
        .withMotionMagicCruiseVelocity(DegreesPerSecond.of(180))
        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(1000))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));
    config.Feedback
        .withSensorToMechanismRatio(kMechanismToMotorGearing);
    config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.25));
    config.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Seconds.of(0.25));
    config.SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(kHighSoftLimit)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(kLowSoftLimit);
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120));
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }

  private void seedEncoderPosition() {
    Angle mechanismAngle = getAbsoluteMechanismAngle(false);
    Angle motorAngle = mechanismAngle;
    m_motor.setPosition(motorAngle);
  }

  private Angle getAbsoluteMechanismAngle(boolean excludeOffset) {
    // Set this to the value of "IntakePivot/intakeAbsoluteEncoderOffset" when the
    // intake is all the way down.
    final double kWrapOffset = -8.9;
    // The number of degress that should be allocated to represnent mechanism position below 0.
    // This needs to be less than 360 divided by the kMechanismToEncoderGearing;
    final double kDegsAllocatedBelow0 = 25;

    double encoderDeg = m_encoder.getAbsolutePosition().getValue().in(Degrees);
    if (encoderDeg < 0)
      encoderDeg += 360d;
    double armDeg = encoderDeg / kMechanismToEncoderGearing;
    double armDegsPerEncoderRot = 360d / kMechanismToEncoderGearing;

    armDeg = (armDeg - (excludeOffset ? 0 : kWrapOffset) + armDegsPerEncoderRot) % armDegsPerEncoderRot;
    armDeg = armDeg > (armDegsPerEncoderRot - kDegsAllocatedBelow0) ? -(armDegsPerEncoderRot - armDeg) : armDeg;
    return Degrees.of(armDeg);
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    m_log.log("intakeAbsoluteEncoder", getAbsoluteMechanismAngle(false).in(Degrees));
    m_log.log("intakeAbsoluteEncoderOffset", getAbsoluteMechanismAngle(true).in(Degrees));
    m_log.log("rawEncoderValue", m_encoder.getAbsolutePosition().getValue());
    m_log.log("outputVoltage", m_motor.getMotorVoltage().getValueAsDouble());
    m_watchdog.end("dogLogging");

    m_watchdog.start();
    m_talonConnectionAlert.set(!m_motor.isAlive());
    boolean highCurrent = m_motor.getStatorCurrent().getValueAsDouble() > 68;
    m_stallAlert.set(highCurrent);
    m_watchdog.end("updateAlerts");
  }

  public Command setAngleCommand(Angle angle) {
    Angle clampedAngle = Degrees
        .of(MathUtil.clamp(angle.in(Degrees), kLowSoftLimit.in(Degrees), kHighSoftLimit.in(Degrees)));
    return Commands.run(() -> {
      m_motor.setControl(m_positionRequest.withPosition(clampedAngle));
    }, this);
  }

  public Angle getAngle() {
    return m_motor.getPosition().getValue();
  }
}