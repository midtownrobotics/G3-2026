package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter {

  private final SparkMax shooterMotor;
  private final SparkMax reverseShooterMotor;
  private final FlyWheel shooterMech;
  private final FlyWheel rshooterMech;
  private final DutyCycleEncoder reverseShooterMotorEncoder; //Use if speed getter sucks
  private final DutyCycleEncoder shooterMotorEncoder; //Use if speed getter sucks

  public Shooter(int shooterMotorID, int shooterMotorEncoderID, int reverseShooterMotorID,
      int reverseShooterMotorEncoderID) {
    shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    shooterMotorEncoder = new DutyCycleEncoder(shooterMotorEncoderID);
    reverseShooterMotor = new SparkMax(reverseShooterMotorEncoderID, MotorType.kBrushless);
    reverseShooterMotorEncoder = new DutyCycleEncoder(reverseShooterMotorEncoderID);

    SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.SHOOTER_GEAR_REDUCTION)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.SHOOTER_P, TurretConstants.SHOOTER_I, TurretConstants.SHOOTER_D,
            TurretConstants.SHOOTER_MAX_ANGULAR_VELOCITY, DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withStatorCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT);

    SmartMotorControllerConfig reverseShooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.SHOOTER_GEAR_REDUCTION)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.SHOOTER_P, TurretConstants.SHOOTER_I, TurretConstants.SHOOTER_D,
            TurretConstants.SHOOTER_MAX_ANGULAR_VELOCITY, DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withStatorCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT)
        .withMotorInverted(true);

    SparkWrapper shooterSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNEO(1), shooterMotorConfig);
    SparkWrapper reverseShooterSmartMotorController = new SparkWrapper(reverseShooterMotor, DCMotor.getNEO(1),
        reverseShooterMotorConfig);

    FlyWheelConfig shooterFlywheelConfig = new FlyWheelConfig(shooterSmartMotorController)
        .withDiameter(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER)
        .withMass(TurretConstants.SHOOTER_MASS)
        .withMOI(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER, TurretConstants.SHOOTER_MASS);

    FlyWheelConfig reverseShooterFlywheelConfig = new FlyWheelConfig(reverseShooterSmartMotorController)
        .withDiameter(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER)
        .withMass(TurretConstants.SHOOTER_MASS)
        .withMOI(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER, TurretConstants.SHOOTER_MASS);

    shooterMech = new FlyWheel(shooterFlywheelConfig);
    rshooterMech = new FlyWheel(reverseShooterFlywheelConfig);
  }

  public AngularVelocity getSpeed() {
    return shooterMech.getSpeed();
  }

  public Command setSpeed(AngularVelocity speed) {
    return Commands.sequence(shooterMech.set(speed.baseUnitMagnitude()), rshooterMech.set(speed.baseUnitMagnitude()));
  }

  public void periodic() {

  }
}
