package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
  private final TalonFX m_pitchMotor;
  private final Arm m_pitchArm;

  public Hood(int pitchMotorID) {
    m_pitchMotor = new TalonFX(pitchMotorID);

    SmartMotorControllerConfig pitchMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kPitchP, TurretConstants.kPitchI, TurretConstants.kPitchD,
            TurretConstants.kPitchMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(260)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Pitch Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate));

    // Assumed pitch motor is NEO
    SmartMotorController pitchMotorController = new TalonFXWrapper(m_pitchMotor, DCMotor.getKrakenX44(16),
        pitchMotorConfig);

    ArmConfig pitchMotorArmConfig = new ArmConfig(pitchMotorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(0), TurretConstants.kPitchPivotHardLimit)
        .withTelemetry("Pitch Arm", TelemetryVerbosity.HIGH)
        .withLength(TurretConstants.kPitchPivotDiameter)
        .withMass(TurretConstants.kPitchPivotMass);

    m_pitchArm = new Arm(pitchMotorArmConfig);
  }

  public void periodic() {

  }

  public Angle getPitchAngle() {
    return m_pitchArm.getAngle();
  }

  public Command setPitchAngleCommand(Angle angle) {
    return m_pitchArm.setAngle(angle);
  }
}
