package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter {

  private final SparkMax shooterMotor;
  private final FlyWheel shooterMech;
  private final DutyCycleEncoder shooterMotorEncoder; //Use if speed getter sucks

  public Shooter(int shooterMotorID, int shooterMotorEncoderID) {
    shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    shooterMotorEncoder = new DutyCycleEncoder(shooterMotorEncoderID);

    SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.SHOOTER_GEAR_REDUCTION)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.SHOOTER_P, TurretConstants.SHOOTER_I, TurretConstants.SHOOTER_D,
            RPM.of(5000), DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.SHOOTER_RAMP_RATE))
        .withStatorCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT);

    SparkWrapper shooterSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNEO(1), shooterMotorConfig);

    FlyWheelConfig shooterFlywheelConfig = new FlyWheelConfig(shooterSmartMotorController)
        .withDiameter(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER)
        .withMass(TurretConstants.SHOOTER_MASS)
        .withMOI(TurretConstants.SHOOTER_FLYWHEEL_DIAMETER, TurretConstants.SHOOTER_MASS);

    shooterMech = new FlyWheel(shooterFlywheelConfig);
  }

  public AngularVelocity getSpeed() {
    return shooterMech.getSpeed();
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooterMech.setSpeed(speed);
  }

  public void periodic() {

  }
}
