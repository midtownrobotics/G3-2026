package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

@Logged(strategy = Strategy.OPT_IN)
public class Turret extends SubsystemBase {
  private final Pivot m_mechanism;
  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;
  private final EasyCRT m_easyCRTSolver;

  public Turret() {
    TalonFX motor = new TalonFX(Ports.kTurretYaw.canId(), Ports.kTurretYaw.canbus());
    m_encoder1 = new CANcoder(Ports.kTurretYawEncoder1.canId(), Ports.kTurretYawEncoder1.canbus());
    m_encoder2 = new CANcoder(Ports.kTurretYawEncoder2.canId(), Ports.kTurretYawEncoder2.canbus());

    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(10, 0, 0,
            DegreesPerSecond.of(950), DegreesPerSecondPerSecond.of(30))
        .withGearing(48)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(30))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorConfig);

    PivotConfig pivotConfig = new PivotConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(-255), Degrees.of(255))
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMOI(KilogramSquareMeters.of(0.1457345474));

    m_mechanism = new Pivot(pivotConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    m_encoder1.getConfigurator().apply(encoderConfig);
    m_encoder2.getConfigurator().apply(encoderConfig);

    Supplier<Angle> encoder1PositionSupplier = () -> m_encoder1.getAbsolutePosition().getValue();
    Supplier<Angle> encoder2PositionSupplier = () -> m_encoder2.getAbsolutePosition().getValue();

    EasyCRTConfig easyCRTConfig = new EasyCRTConfig(encoder1PositionSupplier, encoder2PositionSupplier)
        .withEncoderRatios(0.0, 0.0)
        .withAbsoluteEncoderInversions(false, false)
        .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0));

    m_easyCRTSolver = new EasyCRT(easyCRTConfig);
    m_easyCRTSolver.getAngleOptional().ifPresent(angle -> motorController.setEncoderPosition(angle));
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
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
    Supplier<Angle> newAngle = mapSupplier(angle, this::findNearestAngle);
    return m_mechanism.setAngle(newAngle);
  }

  private Angle findNearestAngle(Angle angle) {
    double targetDegrees = angle.in(Degrees);
    double currentDegrees = getAngle().in(Degrees);

    // Normalize target relative to current angle, then clamp to limits
    double delta = ((targetDegrees - currentDegrees) % 360 + 540) % 360 - 180;
    double bestAngle = currentDegrees + delta;

    // Clamp to turret limits
    if (bestAngle > 255.0) {
      bestAngle = bestAngle - 360.0;
    } else if (bestAngle < -255.0) {
      bestAngle = bestAngle + 360.0;
    }

    // In case we're waaaaay wrong
    bestAngle = MathUtil.clamp(bestAngle, -255, 255);

    return Degrees.of(bestAngle);
  }

  private static <T> Supplier<T> mapSupplier(Supplier<T> supplier, Function<T, T> mapper) {
    return () -> mapper.apply(supplier.get());
  }
}
