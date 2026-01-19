package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

@Logged(strategy = Strategy.OPT_IN)

public class IntakePivot extends SubsystemBase {
  private final SmartMotorController pivotMotor;
  private final Arm pivotArm;

  public IntakePivot() {
    SmartMotorControllerConfig pivotCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(30, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withSimClosedLoopController(30, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.00))
        .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(100, 1)))
        .withStatorCurrentLimit(Amps.of(40));

    SparkMax pivotSpark = new SparkMax(4, MotorType.kBrushless);
    pivotMotor = new SparkWrapper(pivotSpark, DCMotor.getNEO(1), pivotCfg);

    ArmConfig armCfg = new ArmConfig(pivotMotor)
        .withSoftLimits(Degrees.of(-20), Degrees.of(150))
        .withHardLimit(Degrees.of(-20), Degrees.of(150))
        .withStartingPosition(Degrees.of(0))
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("PivotArm", TelemetryVerbosity.HIGH);

    pivotArm = new Arm(armCfg);
  }

  @Override
  public void periodic() {
    pivotArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    pivotArm.simIterate();
  }

  public Command setAngleCommand(Angle angle) {
    return pivotArm.setAngle(angle);
  }
}
