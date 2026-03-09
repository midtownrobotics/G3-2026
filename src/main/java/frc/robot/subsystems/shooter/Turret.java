package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PhoenixUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {
  private static final double kMechanismToMotorGearing = (82d/10d)*(60d/12d);
  private static final Angle kLowSoftLimit = Degrees.of(-255);
  private static final Angle kHighSoftLimit = Degrees.of(255);
  
  private final TalonFX m_motor;
  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;
  private final Watchdawg m_watchdog;

  private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

  public Turret() {
    m_motor = new TalonFX(Ports.kTurretYaw.canId(), Ports.kTurretYaw.canbus());
    m_encoder1 = new CANcoder(Ports.kTurretYawEncoder1.canId(), Ports.kTurretYawEncoder1.canbus());
    m_encoder2 = new CANcoder(Ports.kTurretYawEncoder2.canId(), Ports.kTurretYawEncoder2.canbus());
    m_watchdog = new Watchdawg(getClass());

    configureMotor();
    seedPosition();
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0
      .withKP(10)
      .withKD(0);  


    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    config.MotionMagic
      .withMotionMagicCruiseVelocity(DegreesPerSecond.of(950).times(kMechanismToMotorGearing))
      .withMotionMagicAcceleration(DegreesPerSecond.of(30).per(Seconds).times(kMechanismToMotorGearing));

    
    config.CurrentLimits
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(Amps.of(90));
    config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.25));
    config.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Seconds.of(0.25));

    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(kHighSoftLimit.times(kMechanismToMotorGearing))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(kLowSoftLimit.times(kMechanismToMotorGearing));

    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
  }

  private void seedPosition() {
    Supplier<Angle> encoder1PositionSupplier = () -> m_encoder1.getAbsolutePosition().getValue();
    Supplier<Angle> encoder2PositionSupplier = () -> m_encoder2.getAbsolutePosition().getValue(); 
    EasyCRTConfig CRTConfig = new EasyCRTConfig(encoder1PositionSupplier, encoder2PositionSupplier)
      .withEncoderRatios(0, 0)
      .withAbsoluteEncoderInversions(false, false);

    EasyCRT solver = new EasyCRT(CRTConfig);
    solver.getAngleOptional().ifPresent(angle -> m_motor.setPosition(angle.times(kMechanismToMotorGearing)));
  }

  public Angle getAngle() {
    return m_motor.getPosition().getValue().div(kMechanismToMotorGearing);
  }

  public Command setAngleCommand(Angle angle) {
    angle = findNearestAngle(angle);
    Angle clampedAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kLowSoftLimit.in(Degrees), kHighSoftLimit.in(Degrees)));
    Angle motorAngle = clampedAngle.times(kMechanismToMotorGearing);
    return Commands.run(() -> {
      m_motor.setControl(m_positionRequest.withPosition(motorAngle));
    });
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    final Supplier<Angle> nearestAngle = mapSupplier(angle, this::findNearestAngle);
    return Commands.run(() -> {
      Angle clampedAngle = Degrees.of(MathUtil.clamp(nearestAngle.get().in(Degrees), kLowSoftLimit.in(Degrees), kHighSoftLimit.in(Degrees)));
      Angle motorAngle = clampedAngle.times(kMechanismToMotorGearing);
      m_motor.setControl(m_positionRequest.withPosition(motorAngle));
    });
  }
  
  private Angle findNearestAngle(Angle angle) {
    double targetDegrees = angle.in(Degrees);
    double currentDegrees = getAngle().in(Degrees);

    double delta = ((targetDegrees - currentDegrees) % 360 + 540) % 360 - 180;
    double bestAngle = currentDegrees + delta;

    if (bestAngle > 255.0) {
      bestAngle = bestAngle - 360.0;
    } else if (bestAngle < -255.0) {
      bestAngle = bestAngle + 360.0;
    }

    bestAngle = MathUtil.clamp(bestAngle, -255, 255);

    return Degrees.of(bestAngle);
  }

  private static <T> Supplier<T> mapSupplier(Supplier<T> supplier, Function<T, T> mapper) {
    return () -> mapper.apply(supplier.get());
  }
}