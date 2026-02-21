package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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

public class Hood extends SubsystemBase {
  private final Arm m_mechanism;
  private final CANcoder m_encoder;

  public Hood() {
    TalonFX motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
    m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(250, 0, 0,
            RPM.of(300), RPM.of(500).per(Seconds))
        .withGearing(266)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Hood Motor", TelemetryVerbosity.HIGH)
        .withFeedforward(new ArmFeedforward(0.01, 0, 0))
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withExternalEncoder(m_encoder)
        .withExternalEncoderGearing(19)
        .withUseExternalFeedbackEncoder(true);

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1),
        motorControllerConfig);

    ArmConfig armConfig = new ArmConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(0), Degrees.of(59))
        .withTelemetry("Hood Arm", TelemetryVerbosity.HIGH);

    m_mechanism = new Arm(armConfig);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Angle getAngle() {
    return m_mechanism.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    return m_mechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return m_mechanism.setAngle(angle);
  }
}
