package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class IntakeRoller extends SubsystemBase {
  private final SmartMotorController m_rollerMotor;
  private final FlyWheel m_roller;

  public IntakeRoller() {
    SmartMotorControllerConfig rollerMotorCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
        .withGearing(1)
        .withStatorCurrentLimit(Amps.of(40));

    SparkMax rollerSpark = new SparkMax(5, MotorType.kBrushless);
    m_rollerMotor = new SparkWrapper(rollerSpark, DCMotor.getNEO(1), rollerMotorCfg);

    FlyWheelConfig rollerConfig = new FlyWheelConfig(m_rollerMotor)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withDiameter(Inches.of(1.5))
        .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    m_roller = new FlyWheel(rollerConfig);
  }

  @Override
  public void periodic() {
    m_roller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_roller.simIterate();
  }

  public Command setSpeedCommand(double dutyCycle) {
    return m_roller.set(dutyCycle);
  }
}
