package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {
  private final SparkMax pitchMotor;
  private final Pivot pitchPivot;
  private final DutyCycleEncoder pitchMotorEncoder; //Use if angle getter sucks

  public Hood(int pitchMotorID, int pitchMotorEncoderID) {
    pitchMotor = new SparkMax(pitchMotorID, MotorType.kBrushless);
    pitchMotorEncoder = new DutyCycleEncoder(pitchMotorEncoderID);

    SmartMotorControllerConfig pitchMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.PITCH_P, TurretConstants.PITCH_I, TurretConstants.PITCH_D,
            TurretConstants.PITCH_MOTOR_MAX_ANGULAR_VELOCITY, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.PITCH_GEAR_REDUCTION)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Pitch Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.PITCH_PID_RAMP_RATE))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.PITCH_PID_RAMP_RATE));

    // Assumed pitch motor is NEO
    SmartMotorController pitchMotorController = new SparkWrapper(pitchMotor, DCMotor.getNEO(1), pitchMotorConfig);

    PivotConfig pitchMotorPivotConfig = new PivotConfig(pitchMotorController)
        .withStartingPosition(getPitchAngle())
        .withHardLimit(Degrees.of(0), TurretConstants.PITCH_PIVOT_HARD_LIMIT)
        .withTelemetry("Pitch Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.PITCH_PIVOT_DIAMETER, TurretConstants.PITCH_PIVOT_MASS);

    pitchPivot = new Pivot(pitchMotorPivotConfig);
  }

  public Angle getPitchAngle() {
    return pitchPivot.getAngle();
  }

  public Command setPitchAngleCommand(Angle angle) {
    return pitchPivot.setAngle(angle);
  }

  public void periodic() {

  }
}
