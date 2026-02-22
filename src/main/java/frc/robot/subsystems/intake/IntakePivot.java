package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)

public class IntakePivot extends SubsystemBase {
  private final Arm m_mechanism;
  private final CANcoder m_encoder;

  public IntakePivot() {
    m_encoder = new CANcoder(Ports.kIntakePivotEncoder.canId(), Ports.kIntakePivotEncoder.canbus());

    TalonFX motor = new TalonFX(Ports.kIntakePivot.canId(), Ports.kIntakePivot.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.6, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSimClosedLoopController(3.0, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0.1, 0.4, 0.01))
        .withGearing(48)
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE);

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorControllerConfig);

    ArmConfig armCfg = new ArmConfig(motorController)
        .withHardLimit(Degrees.of(87), Degrees.of(15.25))
        .withStartingPosition(m_encoder.getAbsolutePosition().getValue())
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    m_mechanism = new Arm(armCfg);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    m_encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Command setAngleCommand(Angle angle) {
    return m_mechanism.setAngle(angle);
  }

  @Logged
  public Angle getAngle() {
    return m_mechanism.getAngle();
  }
}
