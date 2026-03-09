package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class Shooter extends SubsystemBase {
  private final FlyWheel m_mechanism;
  private final Alert m_talon1ConnectionAlert = new Alert("Shooter TalonFX motor 1 is not connected",
      AlertType.kWarning);
  private final Alert m_talon2ConnectionAlert = new Alert("Shooter TalonFX motor 2 is not connected",
      AlertType.kWarning);
  private final Alert m_stallAlert1 = new Alert("Shooter motor 1 stalling", AlertType.kWarning);
  private final Alert m_stallAlert2 = new Alert("Shooter motor 2 stalling", AlertType.kWarning);

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private final Watchdawg m_watchdog;

  public Shooter() {
    m_motor1 = new TalonFX(Ports.kTurretShooter1.canId(), Ports.kTurretShooter1.canbus());
    m_motor2 = new TalonFX(Ports.kTurretShooter2.canId(), Ports.kTurretShooter2.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withIdleMode(MotorMode.COAST)
        .withGearing(2d / 3d)
        .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.3, 0, 0,
            RPM.of(10000), RPM.of(1000).per(Second))
        .withFeedforward(new SimpleMotorFeedforward(0, 0.083, 0))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withSupplyCurrentLimit(Amps.of(60))
        .withStatorCurrentLimit(Amps.of(90))
        .withFollowers(Pair.of(m_motor2, true));

    SmartMotorController motorController = new TalonFXWrapper(m_motor1, DCMotor.getKrakenX60(1),
        motorControllerConfig);

    FlyWheelConfig flywheelConfig = new FlyWheelConfig(motorController)
        .withMOI(KilogramSquareMeters.of(0.0021175394))
        .withTelemetry("Shooter", TelemetryVerbosity.LOW);

    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.PeakReverseDutyCycle = 0;
    m_motor1.getConfigurator().apply(outputConfigs);
    m_mechanism = new FlyWheel(flywheelConfig);
    m_watchdog = new Watchdawg(getClass());
  }

  @Override
  public void periodic() {
    m_watchdog.start();
    m_mechanism.updateTelemetry();
    m_watchdog.end("updateTelemetry");

    m_watchdog.start();
    m_talon1ConnectionAlert.set(!m_motor1.isAlive());
    m_talon2ConnectionAlert.set(!m_motor2.isAlive());
    boolean motor1HighCurrent = m_motor1.getStatorCurrent().getValueAsDouble() > 68;
    boolean motor1NotMoving = Math.abs(m_motor1.getVelocity().getValueAsDouble()) < 2;
    boolean motor2HighCurrent = m_motor2.getStatorCurrent().getValueAsDouble() > 68;
    boolean motor2NotMoving = Math.abs(m_motor2.getVelocity().getValueAsDouble()) < 2;
    m_stallAlert1.set(motor1HighCurrent && motor1NotMoving);
    m_stallAlert2.set(motor2HighCurrent && motor2NotMoving);
    m_watchdog.end("updateAlerts");
  }

  private void configureShooter() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = new Slot0Configs()
        .withKP(0.3)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0.083)
        .withKA(0);
    config.MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    config.MotionMagic
        .withMotionMagicCruiseVelocity(RPM.of(9999).times(kMechanismMotorGearing))
        .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(1600).times(kMechanismMotorGearing));
    config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.25));
    config.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Seconds.of(0.25));
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(90));
    PhoenixUtil.tryUntilOk(5, () -> m_motor1.getConfigurator().apply(config));
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public AngularVelocity getSpeed() {
    return m_motor1.getVelocity().getValue().div(kMechanismMotorGearing);
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return Commands.run(() -> {
      m_motor1.setControl(m_velocityRequest.withVelocity(speed.div(kMechanismMotorGearing)));
    });
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> speedSupplier) {
    return Commands.run(() -> {
      m_motor1.setControl(m_velocityRequest.withVelocity(speedSupplier.get().div(kMechanismMotorGearing)));
    });
  }
}
