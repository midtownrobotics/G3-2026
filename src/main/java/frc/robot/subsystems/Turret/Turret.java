package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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

  public Turret(int yawMotorID, int yawMotorEncoderID) {
    yawMotor = new SparkMax(yawMotorID, MotorType.kBrushless);

    SmartMotorControllerConfig yawMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kYawP, TurretConstants.kYawI, TurretConstants.kYawD,
            TurretConstants.kYawMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kYawGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate));

    // Assumed yaw motor is NEO
    SmartMotorController yawMotorController = new SparkWrapper(yawMotor, DCMotor.getKrakenX60(1), yawMotorConfig);

    PivotConfig yawMotorPivotConfig = new PivotConfig(yawMotorController)
        .withStartingPosition(getYawAngle())
        .withWrapping(Degrees.of(0), Degrees.of(360))
        .withHardLimit(Degrees.of(0), TurretConstants.kYawPivotHardLimit)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.kYawPivotDiameter, TurretConstants.kYawPivotMass);

    yawPivot = new Pivot(yawMotorPivotConfig);
  }

  public void periodic() {

  }

  public Angle getYawAngle() {
    return yawPivot.getAngle();
  }

  public Command setYawAngleCommand(Angle angle) {
    return yawPivot.setAngle(angle);
  }
}
