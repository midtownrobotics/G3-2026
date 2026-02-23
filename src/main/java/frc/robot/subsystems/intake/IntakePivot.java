package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
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
  private final Arm m_mechanism;
  private final CANcoder m_encoder;

  public IntakePivot() {
    TalonFX motor = new TalonFX(Ports.kIntakePivot.canId(), Ports.kIntakePivot.canbus());
    m_encoder = new CANcoder(Ports.kIntakePivotEncoder.canId(), Ports.kIntakePivotEncoder.canbus());

    SmartMotorControllerConfig motorControllerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(70.0, 0.0, 0.0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(1000))
        .withSimClosedLoopController(3.0, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(1000))
        .withFeedforward(new ArmFeedforward(0.0, 0.5, 0.0))
        .withGearing(new MechanismGearing(GearBox.fromStages("50:12", "60:20", "48:16")))
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(true)
        .withIdleMode(MotorMode.BRAKE);

    SmartMotorController motorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), motorControllerConfig);

    ArmConfig armConfig = new ArmConfig(motorController)
        .withHardLimit(Degrees.of(0), Degrees.of(70))
        .withSoftLimits(Degrees.of(0), Degrees.of(60))
        .withStartingPosition(getAbsoluteAngle())
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder.getConfigurator().apply(canCoderConfig);

    m_mechanism = new Arm(armConfig);

    var mmConfigs = new MotionMagicConfigs();
    motor.getConfigurator().refresh(mmConfigs);
    mmConfigs.MotionMagicJerk = 10;
    motor.getConfigurator().apply(mmConfigs);
  }

  private Angle getAbsoluteAngle() {
    // Set this to the value of "Intake/IntakeAbsoluteEncoderOffset" when the intake is all the way down.
    final double WRAP_OFFSET = 4.8;

    double encoderDeg = m_encoder.getAbsolutePosition().getValue().in(Degrees);
    if (encoderDeg < 0)
      encoderDeg += 360.0;
    double armDeg = encoderDeg / 3.0;

    armDeg = (armDeg - WRAP_OFFSET + 120.0) % 120.0;
    armDeg = armDeg > 105 ? -(120 - armDeg) : armDeg;
    return Degrees.of(armDeg);
  }

  private Angle getAbsoluteOffsetAt0() {
    double encoderDeg = m_encoder.getAbsolutePosition().getValue().in(Degrees);
    if (encoderDeg < 0)
      encoderDeg += 360.0;
    double armDeg = encoderDeg / 3.0;

    armDeg = (armDeg + 120.0) % 120.0;
    armDeg = armDeg > 105 ? -(120 - armDeg) : armDeg;
    return Degrees.of(armDeg);
  }

  @Override
  public void periodic() {
    m_mechanism.updateTelemetry();
    DogLog.log("Intake/IntakeAbsoluteEncoder", getAbsoluteAngle().in(Degrees));
    DogLog.log("Intake/IntakeAbsoluteEncoderOffset", getAbsoluteOffsetAt0().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Command setAngleCommand(Angle angle) {
    return m_mechanism.setAngle(angle);
  }

  public Angle getAngle() {
    return m_mechanism.getAngle();
  }

  public Command getSysIDCommand() {
    return m_mechanism.sysId(Volts.of(0.7), Volts.of(0.2).per(Second), Seconds.of(10));
  }
}