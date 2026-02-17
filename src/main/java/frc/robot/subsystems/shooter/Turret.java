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
  private final TalonFX m_motor;
  private final Pivot m_pivotMechanism;
  private final CANcoder m_yawCANCoder1;
  private final CANcoder m_yawCANCoder2;
  private final EasyCRT m_easyCRTSolver;

  public Turret(int motorID, int motorEncoderID) {
    m_motor = new TalonFX(Ports.kTurretShooterMotorTalonFXPort);
    m_yawCANCoder1 = new CANcoder(Ports.kTurretCANCoder1Port);
    m_yawCANCoder2 = new CANcoder(Ports.kTurretCANCoder2Port);

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

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX60(1), motorConfig);

    PivotConfig pivotConfig = new PivotConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(-255), Degrees.of(255))
        .withTelemetry("Turret", TelemetryVerbosity.HIGH)
        .withMOI(KilogramSquareMeters.of(0.1457345474));

    CANcoderConfiguration yawCANCoderConfig = new CANcoderConfiguration();
    m_yawCANCoder1.getConfigurator().apply(yawCANCoderConfig);
    m_yawCANCoder2.getConfigurator().apply(yawCANCoderConfig);

    m_pivotMechanism = new Pivot(pivotConfig);

    Supplier<Angle> CAN1Supplier = () -> Rotations.of(m_yawCANCoder1.getAbsolutePosition().getValueAsDouble());
    Supplier<Angle> CAN2Supplier = () -> Rotations.of(m_yawCANCoder2.getAbsolutePosition().getValueAsDouble());

    EasyCRTConfig easyCRT = new EasyCRTConfig(CAN1Supplier, CAN2Supplier)
        .withEncoderRatios(0.0, 0.0)
        .withAbsoluteEncoderInversions(false, false)
        .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0));

    m_easyCRTSolver = new EasyCRT(easyCRT);
    m_easyCRTSolver.getAngleOptional().ifPresent(angleCRT -> {
      motorController.setEncoderPosition(angleCRT);
    });

  }

  @Override
  public void periodic() {
    m_pivotMechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_pivotMechanism.simIterate();
  }

  @Logged
  public Angle getAngle() {
    return m_pivotMechanism.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    return m_pivotMechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    Supplier<Angle> newAngle = mapSupplier(angle, this::findNearestAngle);
    return m_pivotMechanism.setAngle(newAngle);
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
