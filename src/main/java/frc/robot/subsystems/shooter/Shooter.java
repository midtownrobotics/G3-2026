package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PhoenixUtil;
import frc.robot.constants.Ports;

@Logged(strategy = Strategy.OPT_IN)
public class Shooter extends SubsystemBase {
  private static final double kMechanismMotorGearing = 2d / 3d;
  private final TalonFX m_motor1;
  private final TalonFX m_motor2;
  private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);

  public Shooter() {
    m_motor1 = new TalonFX(Ports.kTurretShooter1.canId(), Ports.kTurretShooter1.canbus());
    m_motor2 = new TalonFX(Ports.kTurretShooter2.canId(), Ports.kTurretShooter2.canbus());
    configureShooter();
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
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback
        .withSensorToMechanismRatio(kMechanismMotorGearing);
    config.MotionMagic
        .withMotionMagicCruiseVelocity(RPM.of(9999))
        .withMotionMagicAcceleration(RPM.of(1600).per(Second));
    config.CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(90));
    PhoenixUtil.tryUntilOk(5, () -> m_motor1.getConfigurator().apply(config));
  }

  public AngularVelocity getSpeed() {
    return m_motor1.getVelocity().getValue();
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return Commands.run(() -> {
      m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));
      m_motor1.setControl(m_velocityRequest.withVelocity(speed));
    });
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> speedSupplier) {
    return Commands.run(() -> {
      m_motor2.setControl(new Follower(m_motor1.getDeviceID(), MotorAlignmentValue.Opposed));
      m_motor1.setControl(m_velocityRequest.withVelocity(speedSupplier.get()));
    });
  }
}