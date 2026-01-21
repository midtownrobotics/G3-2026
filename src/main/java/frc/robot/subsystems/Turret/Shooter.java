package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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
  private final SparkMax m_upperShooterMotor;
  private final SparkMax m_lowerShooterMotor;
  private final FlyWheel m_upperShooterMech;
  private final FlyWheel m_lowerShooterMech;

  public Shooter(int shooterMotorID, int shooterMotorEncoderID, int reverseShooterMotorID,
      int reverseShooterMotorEncoderID) {
    m_upperShooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    m_lowerShooterMotor = new SparkMax(reverseShooterMotorEncoderID, MotorType.kBrushless);

    SmartMotorControllerConfig upperShooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.kShooterGearReduction)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kShooterP, TurretConstants.kShooterI, TurretConstants.kShooterD,
            TurretConstants.kShooterMaxAngularVelocity, DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit);

    SmartMotorControllerConfig lowerShooterMotorConfig = new SmartMotorControllerConfig()
        .withIdleMode(MotorMode.COAST)
        .withGearing(TurretConstants.kShooterGearReduction)
        .withTelemetry("Shooter Motor", TelemetryVerbosity.HIGH)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kShooterP, TurretConstants.kShooterI, TurretConstants.kShooterD,
            TurretConstants.kShooterMaxAngularVelocity, DegreesPerSecondPerSecond.of(0))
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kShooterRampRate))
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withMotorInverted(true);

    SparkWrapper upperShooterSmartMotorController = new SparkWrapper(m_upperShooterMotor, DCMotor.getKrakenX60(1), upperShooterMotorConfig);
    SparkWrapper lowerShooterSmartMotorController = new SparkWrapper(m_lowerShooterMotor, DCMotor.getKrakenX60(1),
        lowerShooterMotorConfig);

    FlyWheelConfig shooterFlywheelConfig = new FlyWheelConfig(upperShooterSmartMotorController)
        .withDiameter(TurretConstants.kShooterFlywheelDiameter)
        .withMass(TurretConstants.kShooterMass)
        .withMOI(TurretConstants.kShooterFlywheelDiameter, TurretConstants.kShooterMass);

    FlyWheelConfig reverseShooterFlywheelConfig = new FlyWheelConfig(lowerShooterSmartMotorController)
        .withDiameter(TurretConstants.kShooterFlywheelDiameter)
        .withMass(TurretConstants.kShooterMass)
        .withMOI(TurretConstants.kShooterFlywheelDiameter, TurretConstants.kShooterMass);

    m_upperShooterMech = new FlyWheel(shooterFlywheelConfig);
    m_lowerShooterMech = new FlyWheel(reverseShooterFlywheelConfig);
  }

  public void periodic() {
    m_upperShooterMech.simIterate();
    m_lowerShooterMech.simIterate();
  }

  public AngularVelocity getSpeed() {
    return m_upperShooterMech.getSpeed();
  }

  public Command setSpeed(AngularVelocity speed) {
    return Commands.sequence(m_upperShooterMech.set(speed.baseUnitMagnitude()), m_lowerShooterMech.set(speed.baseUnitMagnitude()));
  }
}
