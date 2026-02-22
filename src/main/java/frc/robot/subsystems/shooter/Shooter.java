package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {
  private final FlyWheel m_mechanism;

  public Shooter() {
    TalonFX motor1 = new TalonFX(Ports.kTurretShooter1.canId(), Ports.kTurretShooter1.canbus());
    TalonFX motor2 = new TalonFX(Ports.kTurretShooter2.canId(), Ports.kTurretShooter2.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(2d / 3d)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0,
            RPM.of(5000), RPM.of(0).per(Second))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withSupplyCurrentLimit(Amps.of(60))
        .withStatorCurrentLimit(Amps.of(90))
        .withFollowers(Pair.of(motor2, true));

    SmartMotorController motorController = new TalonFXWrapper(motor1, DCMotor.getKrakenX60(1),
        motorControllerConfig);

    FlyWheelConfig flywheelConfig = new FlyWheelConfig(motorController)
        .withMOI(KilogramSquareMeters.of(0.0021175394));

    m_mechanism = new FlyWheel(flywheelConfig);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public AngularVelocity getSpeed() {
    return m_mechanism.getSpeed();
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return m_mechanism.setSpeed(speed);
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> speedSupplier) {
    return m_mechanism.setSpeed(speedSupplier);
  }
}
