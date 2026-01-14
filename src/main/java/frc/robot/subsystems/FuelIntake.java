package frc.robot.subsystems.fuel_intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class FuelIntakeSubsystem extends SubsystemBase {

  public enum Goal {
    STOW(Degrees.of(0), Volts.of(0)),
    INTAKING(Degrees.of(90), Volts.of(12)),
    MANUAL();

    private Angle angle;
    private Voltage rollerVoltage;

    private Goal(Angle angle, Voltage rollerVoltage) {
      this.angle = angle;
      this.rollerVoltage = rollerVoltage;
    }

    private Goal() {
    }

    public Angle getAngle() {
      return this.angle;
    }

    public Voltage getRollerVoltage() {
      return this.rollerVoltage;
    }
  }

  private static final class Constants {
    static final int PIVOT_MOTOR_ID = 10;
    static final int ROLLER_MOTOR_ID = 11;

    static final Angle MIN_ANGLE = Degrees.of(-5);
    static final Angle MAX_ANGLE = Degrees.of(100);

    static final double PIVOT_GEAR_RATIO = 80.0;

    static final double PIVOT_KP = 50.0;
    static final double PIVOT_KI = 0.0;
    static final double PIVOT_KD = 0.0;
    static final double PIVOT_MAX_VELOCITY = 180.0;
    static final double PIVOT_MAX_ACCELERATION = 360.0;

    static final double PIVOT_KS = 0.0;
    static final double PIVOT_KG = 0.5;
    static final double PIVOT_KV = 0.0;

    static final double PIVOT_CURRENT_LIMIT = 40.0;
    static final double ROLLER_CURRENT_LIMIT = 30.0;

    static final Angle AT_GOAL_TOLERANCE = Degrees.of(2.0);
  }

  private Goal currentGoal = Goal.STOW;

  public final Trigger atGoalTrigger = new Trigger(this::atGoal);

  private SmartMechanism pivot;
  private SmartMechanism roller;

  private ArmFeedforward pivotFeedforward;
  private ProfiledPIDController pidController;

  private SmartMotorControllerConfig pivotConfig;
  private SmartMotorControllerConfig rollerConfig;

  public FuelIntakeSubsystem() {
    pivotFeedforward = new ArmFeedforward(Constants.PIVOT_KS, Constants.PIVOT_KG, Constants.PIVOT_KV);

    pidController = new ProfiledPIDController(
        Constants.PIVOT_KP,
        Constants.PIVOT_KI,
        Constants.PIVOT_KD,
        new Constraints(
            Constants.PIVOT_MAX_VELOCITY, Constants.PIVOT_MAX_ACCELERATION));

    pidController.setIntegratorRange(-0.2, 0.2);
    pidController.setIZone(0.2);

    pivotConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
            Constants.PIVOT_KP,
            Constants.PIVOT_KI,
            Constants.PIVOT_KD,
            DegreesPerSecond.of(Constants.PIVOT_MAX_VELOCITY),
            DegreesPerSecondPerSecond.of(Constants.PIVOT_MAX_ACCELERATION))
        .withSimClosedLoopController(
            Constants.PIVOT_KP,
            Constants.PIVOT_KI,
            Constants.PIVOT_KD,
            DegreesPerSecond.of(Constants.PIVOT_MAX_VELOCITY),
            DegreesPerSecondPerSecond.of(Constants.PIVOT_MAX_ACCELERATION))
        .withFeedforward(pivotFeedforward)
        .withSimFeedforward(pivotFeedforward)
        .withTelemetry("FuelIntakePivot", TelemetryVerbosity.HIGH)
        .withGearing(Constants.PIVOT_GEAR_RATIO)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(Constants.PIVOT_CURRENT_LIMIT))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    rollerConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withTelemetry("FuelIntakeRoller", TelemetryVerbosity.MEDIUM)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(Constants.ROLLER_CURRENT_LIMIT))
        .withOpenLoopRampRate(Seconds.of(0.1));

    pivot = new SmartMechanism(Constants.PIVOT_MOTOR_ID, pivotConfig);
    roller = new SmartMechanism(Constants.ROLLER_MOTOR_ID, rollerConfig);

    setGoal(Goal.STOW);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    if (RobotState.isDisabled()) {
      double position = getPosition().in(Radians);
      pidController.setGoal(position);
      pidController.reset(position);
    }

    Angle desiredAngle = currentGoal.getAngle();
    Voltage desiredRollerVoltage = currentGoal.getRollerVoltage();

    Angle constrainedAngle = clampAngle(desiredAngle);

    switch (currentGoal) {
      case STOW:
      case INTAKING:
      case OUTTAKE:
      case HANDOFF:
        pivot.setReference(
            Degrees.of(calculateVoltageForPosition(constrainedAngle).in(Volts)));
        roller.setReference(desiredRollerVoltage.in(Volts) / 12.0);
        break;
      case MANUAL:
        break;
    }
  }

  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  public Goal getCurrentGoal() {
    return currentGoal;
  }

  public Angle getPosition() {
    return Degrees.of(pivot.getPosition().in(Degrees));
  }

  public AngularVelocity getVelocity() {
    return pivot.getVelocity();
  }

  private Angle clampAngle(Angle angle) {
    if (angle.lt(Constants.MIN_ANGLE)) {
      return Constants.MIN_ANGLE;
    } else if (angle.gt(Constants.MAX_ANGLE)) {
      return Constants.MAX_ANGLE;
    }
    return angle;
  }

  private Voltage calculateVoltageForPosition(Angle desired) {
    Voltage pidVoltage = Volts.of(
        pidController.calculate(
            getPosition().in(Radians), desired.in(Radians)));

    TrapezoidProfile.State setpoint = pidController.getSetpoint();
    Voltage ffVoltage = Volts.of(pivotFeedforward.calculate(setpoint.position, setpoint.velocity));

    return pidVoltage.plus(ffVoltage);
  }

  public boolean atGoal() {
    return atGoal(currentGoal);
  }

  public boolean atGoal(Goal goal) {
    return atGoal(goal, Constants.AT_GOAL_TOLERANCE);
  }

  public boolean atGoal(Goal goal, Angle angleTolerance) {
    return currentGoal == goal && getPosition().isNear(goal.getAngle(), angleTolerance);
  }

  public Trigger atGoalTrigger(Goal goal) {
    return new Trigger(() -> atGoal(goal));
  }

  public Trigger atGoalTrigger(Goal goal, Angle angleTolerance) {
    return new Trigger(() -> atGoal(goal, angleTolerance));
  }

  public Supplier<Command> stowCommand() {
    return () -> setGoalCommand(Goal.STOW);
  }

  public Supplier<Command> intakeCommand() {
    return () -> setGoalEndCommand(Goal.INTAKING, Goal.STOW);
  }

  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }

  public Command setGoalAndWait(Goal goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  public Command setGoalAndWait(Goal goal, Angle tolerance) {
    return run(() -> setGoal(goal)).until(() -> atGoal(goal, tolerance));
  }
}
