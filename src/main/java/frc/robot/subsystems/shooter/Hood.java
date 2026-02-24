package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggerUtil;
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
  private final CANcoder m_encoder;

  public Hood() {
    TalonFX motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
    m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(250, 0, 3,
            RPM.of(300), RPM.of(500).per(Seconds))
        .withGearing(266)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
        .withFeedforward(new ArmFeedforward(0.01, 0, 0))
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1),
        motorControllerConfig);

    ArmConfig armConfig = new ArmConfig(motorController)
        .withHardLimit(Degrees.of(0), Degrees.of(59))
        .withSoftLimits(Degrees.of(2), Degrees.of(55))
        .withTelemetry("Hood", TelemetryVerbosity.HIGH)
        .withMOI(KilogramSquareMeters.of(0.038))
        .withLength(Inches.of(6))
        .withStartingPosition(m_encoder.getAbsolutePosition().getValue().div(19));

    m_mechanism = new Arm(armConfig);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
    LoggerUtil.log("encoderPosition", m_encoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  @Logged
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
