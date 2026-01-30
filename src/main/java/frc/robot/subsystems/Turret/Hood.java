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
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
  private final TalonFX m_motor;
  private final Arm m_armMechanism;

  public Hood(int motorID, int encoderID) {
    m_motor = new TalonFX(Ports.kHoodMotor);

    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kPitchP, TurretConstants.kPitchI, TurretConstants.kPitchD,
            TurretConstants.kPitchMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kPitchGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Pitch Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate));

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX44(1),
        motorConfig);

    ArmConfig motorArmConfig = new ArmConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(0), TurretConstants.kPitchPivotHardLimit)
        .withTelemetry("Pitch Arm", TelemetryVerbosity.HIGH);

    m_armMechanism = new Arm(motorArmConfig);
  }

  public void periodic() {

  }

  public Angle getAngle() {
    return m_armMechanism.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    return m_armMechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return m_armMechanism.setAngle(angle);
  }
}
