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
  private final SmartMotorController m_pivotMotor;
  private final Arm m_pivotArm;
  private final CANcoder m_intakeCANCoder;

  public IntakePivot() {
    m_intakeCANCoder = new CANcoder(Ports.kIntakePivotCANPort);
    SmartMotorControllerConfig pivotCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.6, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0.1, 0.4, 0.01))
        .withGearing(48)
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE);

    TalonFX pivotTalonFX = new TalonFX(Ports.kIntakePivotTalonFXPort);
    m_pivotMotor = new TalonFXWrapper(pivotTalonFX, DCMotor.getKrakenX60(1), pivotCfg);

    ArmConfig armCfg = new ArmConfig(m_pivotMotor)
        .withHardLimit(Degrees.of(87), Degrees.of(15.25))
        .withStartingPosition(m_intakeCANCoder.getAbsolutePosition().getValue())
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    m_pivotArm = new Arm(armCfg);

    CANcoderConfiguration m_intakeCANCoderConfiguration = new CANcoderConfiguration();
    m_intakeCANCoderConfiguration.MagnetSensor.MagnetOffset = 0.0;
    m_intakeCANCoder.getConfigurator().apply(m_intakeCANCoderConfiguration);
  }

  @Override
  public void periodic() {
    m_pivotArm.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    m_pivotArm.simIterate();
  }

  public Command setAngleCommand(Angle angle) {
    return m_pivotArm.setAngle(angle);
  }

  public Angle getAngle() {
    return m_pivotArm.getAngle();
  }
}
