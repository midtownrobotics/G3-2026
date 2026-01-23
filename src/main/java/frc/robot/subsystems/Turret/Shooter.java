package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter {
  private final TalonFX m_shooterMotor;
  private final FlyWheel m_shooterMechanism;

  public Shooter(int shooterMotorID, int shooterMotorEncoderID, int reverseShooterMotorID,
      int reverseShooterMotorEncoderID) {
    m_shooterMotor = new TalonFX(6);
    SmartMotorControllerConfig upperShooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.kShooterGearReduction)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kShooterP, TurretConstants.kShooterI, TurretConstants.kShooterD,
            TurretConstants.kShooterMaxAngularVelocity, DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit);

    TalonFXWrapper upperShooterSmartMotorController = new TalonFXWrapper(m_shooterMotor, DCMotor.getKrakenX60(1),
        upperShooterMotorConfig);

    FlyWheelConfig shooterFlywheelConfig = new FlyWheelConfig(upperShooterSmartMotorController)
        .withDiameter(TurretConstants.kShooterFlywheelDiameter)
        .withMass(TurretConstants.kShooterMass)
        .withMOI(TurretConstants.kShooterFlywheelDiameter, TurretConstants.kShooterMass);

    m_shooterMechanism = new FlyWheel(shooterFlywheelConfig);
  }

  public void periodic() {
    m_shooterMechanism.simIterate();
  }

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
