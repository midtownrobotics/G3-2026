package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Yaw extends SubsystemBase {
  private final TalonFX m_motor;
  private final Pivot m_pivotMechanism;

  public Yaw(int motorID, int motorEncoderID) {
    m_motor = new TalonFX(Ports.kTurretMotor);

    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kYawP, TurretConstants.kYawI, TurretConstants.kYawD,
            TurretConstants.kYawMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kYawGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate));

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX60(1), motorConfig);

    PivotConfig motorPivotConfig = new PivotConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withWrapping(Degrees.of(0), Degrees.of(360))
        .withHardLimit(Degrees.of(0), TurretConstants.kYawPivotHardLimit)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.kYawPivotDiameter, TurretConstants.kYawPivotMass);

    m_pivotMechanism = new Pivot(motorPivotConfig);
  }

  @Override
  public void periodic() {
    m_pivotMechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMechanism.simIterate();
  }

  public Angle getAngle() {
    return m_pivotMechanism.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    return m_pivotMechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return m_pivotMechanism.setAngle(angle);
  }
}
