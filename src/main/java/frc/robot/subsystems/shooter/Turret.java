package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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

public class Turret extends SubsystemBase {
  private final TalonFX m_motor;
  private final Pivot m_pivotMechanism;
  private final CANcoder m_yawCANCoder1;
  private final CANcoder m_yawCANCoder2;
  private final EasyCRT m_easyCRTSolver;

  public Turret(int motorID, int motorEncoderID) {
    m_motor = new TalonFX(Ports.kTurretYawMotorTalonFXPort);
    m_yawCANCoder1 = new CANcoder(Ports.kTurretYawCANPort1);
    m_yawCANCoder2 = new CANcoder(Ports.kTurretYawCANPort2);

    SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kYawP, TurretConstants.kYawI, TurretConstants.kYawD,
            TurretConstants.kYawMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kYawGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate));

    SmartMotorController motorController = new TalonFXWrapper(m_motor, DCMotor.getKrakenX60(1), motorConfig);

    PivotConfig motorPivotConfig = new PivotConfig(motorController)
        .withStartingPosition(Degrees.of(0))
        .withHardLimit(Degrees.of(0), TurretConstants.kYawPivotHardLimit)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.kYawPivotDiameter, TurretConstants.kYawPivotMass);

    CANcoderConfiguration yawCANCoderConfig = new CANcoderConfiguration();
    m_yawCANCoder1.getConfigurator().apply(yawCANCoderConfig);
    m_yawCANCoder2.getConfigurator().apply(yawCANCoderConfig);

    m_pivotMechanism = new Pivot(motorPivotConfig);

    Supplier<Angle> CAN1Supplier = () -> Rotations.of(m_yawCANCoder1.getAbsolutePosition().getValueAsDouble());
    Supplier<Angle> CAN2Supplier = () -> Rotations.of(m_yawCANCoder2.getAbsolutePosition().getValueAsDouble());

    double kYawCANCoder1Ratio = 0;
    double kYawCANCoder2Ratio = 0;

    EasyCRTConfig easyCRT = new EasyCRTConfig(CAN1Supplier, CAN2Supplier)
        .withEncoderRatios(kYawCANCoder1Ratio, kYawCANCoder2Ratio)
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

  public Angle getAngle() {
    return m_pivotMechanism.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    return m_pivotMechanism.setAngle(angle);
  }

  public Command setAngleCommand(Supplier<Angle> angle) {
    return m_pivotMechanism.setAngle(angle);
  }
}
