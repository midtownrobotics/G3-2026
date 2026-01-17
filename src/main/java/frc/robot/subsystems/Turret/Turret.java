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

public class Turret extends SubsystemBase {
  private final SparkMax yawMotor;
  private final Pivot yawPivot;
  private final DutyCycleEncoder yawMotorEncoder; //Use if angle getter sucks

  public Turret(int yawMotorID, int yawMotorEncoderID) {
    yawMotor = new SparkMax(yawMotorID, MotorType.kBrushless);
    yawMotorEncoder = new DutyCycleEncoder(yawMotorEncoderID);

    SmartMotorControllerConfig yawMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.YAW_P, TurretConstants.YAW_I, TurretConstants.YAW_D,
            TurretConstants.YAW_MOTOR_MAX_ANGULAR_VELOCITY, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.YAW_GEAR_REDUCTION)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.YAW_PID_RAMP_RATE))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.YAW_PID_RAMP_RATE));

    // Assumed yaw motor is NEO
    SmartMotorController yawMotorController = new SparkWrapper(yawMotor, DCMotor.getNEO(1), yawMotorConfig);

    PivotConfig yawMotorPivotConfig = new PivotConfig(yawMotorController)
        .withStartingPosition(getYawAngle())
        .withWrapping(Degrees.of(0), Degrees.of(360))
        .withHardLimit(Degrees.of(0), TurretConstants.YAW_PIVOT_HARD_LIMIT)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.YAW_PIVOT_DIAMETER, TurretConstants.YAW_PIVOT_MASS);

    yawPivot = new Pivot(yawMotorPivotConfig);
  }

  public Angle getYawAngle() {
    return yawPivot.getAngle();
  }

  public Command setYawAngleCommand(Angle angle) {
    return yawPivot.setAngle(angle);
  }

  public void periodic() {

  }
}
