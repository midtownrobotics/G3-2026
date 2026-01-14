package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FuelIntake extends SubsystemBase {

  private final SmartMotorController pivotMotor;
  private final SmartMotorController rollerMotor;
  private final Arm pivotArm;

  public enum Goal {
    //these could be the values i have truly no idea
    STOW(0.0, 0.0),
    INTAKING(90.0, 12.0);

    private final double angleDeg;
    private final double rollerVoltage;

    Goal(double angleDeg, double rollerVoltage) {
      this.angleDeg = angleDeg;
      this.rollerVoltage = rollerVoltage;
    }

    public double getAngleDeg() {
      return angleDeg;
    }

    public double getRollerVoltage() {
      return rollerVoltage;
    }
  }

  private Goal currentGoal = Goal.STOW;

  public FuelIntake() {
    SmartMotorControllerConfig pivotCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.6, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0.1, 0.4, 0.01))
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE);

    SparkMax pivotSpark = new SparkMax(4, MotorType.kBrushless);
    pivotMotor = new SparkWrapper(pivotSpark, DCMotor.getNEO(1), pivotCfg);

    SmartMotorControllerConfig rollerCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH);

    SparkMax rollerSpark = new SparkMax(5, MotorType.kBrushless);
    rollerMotor = new SparkWrapper(rollerSpark, DCMotor.getNEO(1), rollerCfg);

    ArmConfig armCfg = new ArmConfig(pivotMotor)
        .withSoftLimits(Degrees.of(-20), Degrees.of(150))
        .withStartingPosition(Degrees.of(0))
        .withLength(Feet.of(1.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    pivotArm = new Arm(armCfg);
  }

  @Override
  public void periodic() {
    pivotArm.updateTelemetry();
    //move the arm to where we want it to be using currentgoal angle i have no idea what this is right now
    pivotArm.setAngle(Degrees.of(currentGoal.getAngleDeg()));

    rollerMotor.setVoltage(Volts.of(currentGoal.getRollerVoltage()));
  }

  @Override
  public void simulationPeriodic() {
    pivotArm.simIterate();
  }

  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  public Command stowCommand() {
    return runOnce(() -> setGoal(Goal.STOW));
  }

  public Command intakeCommand() {
    return runOnce(() -> setGoal(Goal.INTAKING));
  }

  public boolean atGoal() {
    return Math.abs(pivotArm.getAngle().in(Degrees) - currentGoal.getAngleDeg()) < 2.0;
  }
}
