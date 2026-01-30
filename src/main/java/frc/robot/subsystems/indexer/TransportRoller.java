package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

@Logged(strategy = Strategy.OPT_IN)
public class TransportRoller extends SubsystemBase {
  private final SmartMotorController m_rollerMotor;
  private final FlyWheel m_roller;

  public TransportRoller() {
    SmartMotorControllerConfig rollerMotorCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("TransportRollerMotor", TelemetryVerbosity.HIGH);

    TalonFX rollerTalon = new TalonFX(22);
    m_rollerMotor = new TalonFXWrapper(rollerTalon, DCMotor.getKrakenX60(1), rollerMotorCfg);

    FlyWheelConfig rollerConfig = new FlyWheelConfig(m_rollerMotor)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(5000))
        .withLowerSoftLimit(RPM.of(-5000))
        .withDiameter(Inches.of(1.5))
        .withTelemetry("TransportRoller", TelemetryVerbosity.HIGH);

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

  public Command setSpeedCommand(AngularVelocity speed) {
    return m_roller.setSpeed(speed);
  }

  public Command stopCommand() {
    return m_roller.set(0.0);
  }
}
