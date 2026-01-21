package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
public class Feeder extends SubsystemBase {
  private final SmartMotorController m_feederMotor;
  private final FlyWheel m_feeder;

  public Feeder() {
    SmartMotorControllerConfig beltMotorCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withClosedLoopController(0.3, 0, 0.01)
        .withFeedforward(new SimpleMotorFeedforward(0.05, 0.12, 0))
        .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

    //I have no idea what our canbus ID is
    TalonFX beltTalonFX = new TalonFX(3);
    //also don't know what motors we are using so just put this here for now
    m_feederMotor = new TalonFXWrapper(beltTalonFX, DCMotor.getKrakenX60(1), beltMotorCfg);

    FlyWheelConfig beltConfig = new FlyWheelConfig(m_feederMotor)
        .withMass(Pounds.of(0.5))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
        .withDiameter(Inches.of(1.5))
        .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

    m_feeder = new FlyWheel(beltConfig);
  }

  @Override
  public void periodic() {
    m_feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feeder.simIterate();
  }

  public Command setSpeedCommand(AngularVelocity angularVelocity) {
    return m_feeder.setSpeed(angularVelocity);
  }

  public Command offCommand() {
    return m_feeder.set(0.0);
  }
}
