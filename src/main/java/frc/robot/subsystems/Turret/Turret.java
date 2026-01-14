package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Turret extends SubsystemBase {
  private final SparkMax yawMotor;
  private final SparkMax pitchMotor;
  private final SparkMax shootMotor;
  private final FlyWheel shooterMech;
  private final Pivot yawPivot;
  private final Pivot pitchPivot;
  private final DutyCycleEncoder yawMotorEncoder;
  private final DutyCycleEncoder pitchMotorEncoder;
  private boolean isTargetingHub;

  public Turret(int yawMotorID, int pitchMotorID, int yawMotorEncoderID, int pitchMotorEncoderID, int shootMotorID) {
    yawMotor = new SparkMax(yawMotorID, MotorType.kBrushless);
    pitchMotor = new SparkMax(pitchMotorID, MotorType.kBrushless);
    shootMotor = new SparkMax(shootMotorID, MotorType.kBrushless);
    yawMotorEncoder = new DutyCycleEncoder(yawMotorEncoderID);
    pitchMotorEncoder = new DutyCycleEncoder(pitchMotorEncoderID);

    SmartMotorControllerConfig shootMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.shooterP, TurretConstants.shooterI, TurretConstants.shooterD,
            RPM.of(600), DegreesPerSecondPerSecond.of(60))
        .withIdleMode(MotorMode.COAST)
        /*.withGearing() */
        .withTelemetry("Flywheel Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(TurretConstants.motorCurrentLimit))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.shooterPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.shooterPIDRampRate));

    SmartMotorControllerConfig yawMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.yawP, TurretConstants.yawI, TurretConstants.yawD,
            DegreesPerSecond.of(60), DegreesPerSecondPerSecond.of(30))
        /*.withGearing() */
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(TurretConstants.motorCurrentLimit))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.yawPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.yawPIDRampRate));

    SmartMotorControllerConfig pitchMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.pitchP, TurretConstants.pitchI, TurretConstants.pitchD,
            DegreesPerSecond.of(60), DegreesPerSecondPerSecond.of(30))
        /*.withGearing() */
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Pitch Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(TurretConstants.motorCurrentLimit))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.pitchPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.pitchPIDRampRate));

    // Assumed pitch and yaw motors are NEOs
    SmartMotorController yawMotorController = new SparkWrapper(yawMotor, DCMotor.getNEO(1), yawMotorConfig);
    SmartMotorController pitchMotorController = new SparkWrapper(pitchMotor, DCMotor.getNEO(1),
        pitchMotorConfig);
    SmartMotorController shooterMotorController = new SparkWrapper(shootMotor, DCMotor.getNEO(1), shootMotorConfig);

    FlyWheelConfig shooterFlyWheelConfig = new FlyWheelConfig(shooterMotorController)
        .withDiameter(Meters.of(TurretConstants.shooterFlyWheelDiameter))
        .withTelemetry("Shooter Mechanism", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(TurretConstants.shooterMass));

    PivotConfig yawMotorPivotConfig = new PivotConfig(yawMotorController)
        .withStartingPosition(getYawAngle())
        .withWrapping(Degrees.of(0), Degrees.of(360))
        .withHardLimit(Degrees.of(0), TurretConstants.yawPivotHardLimit)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
    /* .withMOI() */;

    PivotConfig pitchMotorPivotConfig = new PivotConfig(pitchMotorController)
        .withStartingPosition(getPitchAngle())
        .withHardLimit(Degrees.of(0), TurretConstants.pitchPivotHardLimit)
        .withTelemetry("Pitch Pivot", TelemetryVerbosity.HIGH)
    /* .withMOI() */;

    yawPivot = new Pivot(yawMotorPivotConfig);
    pitchPivot = new Pivot(pitchMotorPivotConfig);
    shooterMech = new FlyWheel(shooterFlyWheelConfig);
  }

  private double getMuzzleVelocity(AngularVelocity speed, Double wheelDiameter) {
    return TurretConstants.shooterVelocityLoss * ((speed.in(RPM) * Math.PI * wheelDiameter) / 60);
  }

  public void targetHub(Boolean target) {
    isTargetingHub = target;
  }

  public boolean isTargetingHub() {
    return isTargetingHub;
  }

  public Angle getYawAngle() {
    return Degrees.of(yawMotorEncoder.get() * 360);
  }

  public Angle getPitchAngle() {
    return Degrees.of(pitchMotorEncoder.get() * 360);
  }

  public Angle yawFromCoords(double leftMeters, double forwardMeters) {
    return Radians.of(Math.atan2(leftMeters, forwardMeters));
  }

  public Angle pitchFromCoords(double leftMeters, double forwardMeters, double upMeters, double launchVelocity) {
    double gravity = 9.80665;
    double v2 = launchVelocity * launchVelocity;
    double horizontalDistance = Math.hypot(forwardMeters, leftMeters);

    if (horizontalDistance < 1e-6) {
      return null; // invalid shot
    }

    double discriminant = v2 * v2 - gravity * (gravity * horizontalDistance * horizontalDistance + 2 * upMeters * v2);

    if (discriminant < 0) {
      return null; // unreachable at this velocity
    }

    return Radians.of(Math.atan2((v2 + Math.sqrt(discriminant)), (gravity * horizontalDistance)));
  }

  public AngularVelocity getShooterSpeed() {
    return shooterMech.getSpeed();
  }

  public void setShooterSpeed(AngularVelocity speed) {
    shooterMech.setSpeed(speed);
  }

  private void setTurretAngle(Angle yaw, Angle pitch) {
    yawPivot.setAngle(yaw);
    pitchPivot.setAngle(pitch);
  }

  public void aimAtTarget(double leftMeters, double forwardMeters, double upMeters) {
    double muzzleVelocity = getMuzzleVelocity(getShooterSpeed(), TurretConstants.shooterFlyWheelDiameter);
    Angle pitch = pitchFromCoords(leftMeters, forwardMeters, upMeters, muzzleVelocity);
    if (pitch == null)
      return;
    Angle yaw = yawFromCoords(leftMeters, forwardMeters);

    setTurretAngle(yaw, pitch);

  }

  public void periodic() {

  }
}
