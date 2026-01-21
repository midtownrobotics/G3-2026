package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {
  private final SparkMax pitchMotor;
  private final Arm pitchArm;

  public Hood(int pitchMotorID, int pitchMotorEncoderID) {
    pitchMotor = new SparkMax(pitchMotorID, MotorType.kBrushless);

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
        

    // Assumed pitch motor is NEO
    SmartMotorController pitchMotorController = new SparkWrapper(pitchMotor, DCMotor.getKrakenX44(1), pitchMotorConfig);

    ArmConfig pitchMotorArmConfig = new ArmConfig(pitchMotorController)
        .withStartingPosition(getPitchAngle())
        .withHardLimit(Degrees.of(0), TurretConstants.kPitchPivotHardLimit)
        .withTelemetry("Pitch Arm", TelemetryVerbosity.HIGH);

    pitchArm = new Arm(pitchMotorArmConfig);
  }

  public void periodic() {

  }

    public Angle getPitchAngle() {
    return pitchArm.getAngle();
  }

  public Command setPitchAngleCommand(Angle angle) {
    return pitchArm.setAngle(angle);
  }
}
