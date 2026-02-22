package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggerUtil;
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
    m_encoder = new CANcoder(Ports.kIntakePivotEncoder.canId(), Ports.kIntakePivotEncoder.canbus());

    TalonFX motor = new TalonFX(Ports.kIntakePivot.canId(), Ports.kIntakePivot.canbus());

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

    ArmConfig armCfg = new ArmConfig(motorController)
         .withHardLimit(Degrees.of(0), Degrees.of(70))
        .withSoftLimits(Degrees.of(0), Degrees.of(60))
        .withStartingPosition(getAbsoluteAngle())
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    m_mechanism = new Arm(armCfg);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = 0.0;
    m_encoder.getConfigurator().apply(encoderConfig);
  }

  private Angle getAbsoluteAngle() {
    // Set this to the value of "IntakePivot/intakeAbsoluteEncoderOffsetValue" when the intake is all the way down.
    final double WRAP_OFFSET = 40;

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
    LoggerUtil.log("intakeAbsoluteEncoder", getAbsoluteAngle());
    LoggerUtil.log("intakeAbsoluteEncoderOffsetValue", getAbsoluteOffsetAt0().in(Degrees));
    m_mechanism.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_mechanism.simIterate();
  }

  public Command setAngleCommand(Angle angle) {
    return m_mechanism.setAngle(angle);
  }

  @Logged
  public Angle getAngle() {
    return m_mechanism.getAngle();
  }
}
