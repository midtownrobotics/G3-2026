package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class Shooter extends SubsystemBase {
  private final TalonFX m_shooterMotor;
  private final FlyWheel m_shooterMechanism;

  public Shooter(int shooterMotorID, int shooterMotorEncoderID, int reverseShooterMotorID,
      int reverseShooterMotorEncoderID) {
    m_shooterMotor = new TalonFX(Ports.kTurretShooterMotorTalonFXPort);

    SmartMotorControllerConfig upperShooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(0)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0, 0, 0,
            RPM.of(5000), RPM.of(0).per(Second))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withSupplyCurrentLimit(Amps.of(60))
        .withStatorCurrentLimit(Amps.of(90));

    TalonFXWrapper upperShooterSmartMotorController = new TalonFXWrapper(m_shooterMotor, DCMotor.getKrakenX60(1),
        upperShooterMotorConfig);

    FlyWheelConfig shooterFlywheelConfig = new FlyWheelConfig(upperShooterSmartMotorController)
        .withMOI(KilogramSquareMeters.of(0.0021175394));

    m_shooterMechanism = new FlyWheel(shooterFlywheelConfig);
  }

  @Override
  public void periodic() {
    m_shooterMechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_shooterMechanism.simIterate();
  }

  @Logged
  public AngularVelocity getSpeed() {
    return m_shooterMechanism.getSpeed();
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return m_shooterMechanism.setSpeed(speed);
  }

  public Command setSpeedCommand(Supplier<AngularVelocity> speedSupplier) {
    return m_shooterMechanism.setSpeed(speedSupplier);
  }
}
