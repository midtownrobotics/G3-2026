package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Logger;
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
public class Hood extends SubsystemBase {
  private final Arm m_mechanism;
  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  private final Logger m_log;
  private final Trigger m_currentSpikeTrigger;
  private final LinearFilter m_currentSpikeFilter;

  public Hood() {
    m_motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
    m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(250, 0, 3,
            RPM.of(300), RPM.of(500).per(Seconds))
        .withGearing(266)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("HoodMotor", TelemetryVerbosity.LOW)
        .withFeedforward(new ArmFeedforward(0.01, 0, 0))
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX44(1),
        motorControllerConfig);

    ArmConfig armConfig = new ArmConfig(motorController)
        .withHardLimit(Degrees.of(0), Degrees.of(59))
        .withSoftLimits(Degrees.of(2), Degrees.of(55))
        .withTelemetry("Hood", TelemetryVerbosity.LOW)
        .withMOI(KilogramSquareMeters.of(0.038))
        .withLength(Inches.of(6))
        .withStartingPosition(m_encoder.getAbsolutePosition().getValue().div(19));

    m_mechanism = new Arm(armConfig);
    m_log = new Logger(getClass());
    m_currentSpikeFilter = LinearFilter.movingAverage(5);
    m_currentSpikeTrigger = new Trigger(this::getIsCurrentSpiking);
  }

  private boolean getIsCurrentSpiking() {
    return Amps.of(m_currentSpikeFilter.calculate(m_mechanism.getMotor().getStatorCurrent().in(Amps))).gt(Amps.of(20));
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
    m_log.log("encoderPosition", m_encoder.getAbsolutePosition().getValueAsDouble());
    m_log.log("ampsDrawn", m_mechanism.getMotor().getStatorCurrent().in(Amps));
    m_log.log("currentSpike", getIsCurrentSpiking());
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  @Logged
  public Angle getAngle() {
    return m_mechanism.getAngle();
  }

  public Command setVoltage(Voltage volts) {
    return Commands.run(() -> m_motor.setVoltage(volts.in(Volts)));
  }

  public Trigger getCurrentSpikeTrigger() {
    return m_currentSpikeTrigger;
  }

  public Trigger isNearTrigger(Supplier<Angle> angle, Angle threshold) {
    return new Trigger(() -> m_mechanism.getAngle().isNear(angle.get(), threshold));
  }

  public Command setEncoderAngleCommand(Angle angle) {
    return Commands.runOnce(() -> m_mechanism.getMotor().setEncoderPosition(angle));
  }

  public Command zeroEncoderAngleCommand() {
    return setEncoderAngleCommand(Degrees.zero());
  }

  public Command setAngleCommand(Angle angle) {
    return m_mechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return m_mechanism.setAngle(angle);
  }
}
