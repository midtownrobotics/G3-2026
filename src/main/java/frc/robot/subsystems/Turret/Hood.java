package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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
  private final CANcoder m_pitchCANCoder;

  public Hood(int pitchMotorID, int pitchMotorEncoderID) {
    m_pitchMotor = new TalonFX(6);
    m_pitchCANCoder = new CANcoder(5);
    SmartMotorControllerConfig pitchMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kPitchP, TurretConstants.kPitchI, TurretConstants.kPitchD,
            TurretConstants.kPitchMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kPitchGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Pitch Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate))

        .withOpenLoopRampRate(Seconds.of(TurretConstants.kPitchPIDRampRate));

    SmartMotorController pitchMotorController = new TalonFXWrapper(m_pitchMotor, DCMotor.getKrakenX44(1),
        pitchMotorConfig);

    ArmConfig pitchMotorArmConfig = new ArmConfig(pitchMotorController)
        .withStartingPosition(m_pitchCANCoder.getAbsolutePosition().getValue())
        .withHardLimit(TurretConstants.kPitchPivotHardMin, TurretConstants.kPitchPivotHardLimit)
        .withTelemetry("Pitch Arm", TelemetryVerbosity.HIGH);

    m_pitchArm = new Arm(pitchMotorArmConfig);

    CANcoderConfiguration pitchCANCoderConfig = new CANcoderConfiguration();
    m_pitchCANCoder.getConfigurator().apply(pitchCANCoderConfig);
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
