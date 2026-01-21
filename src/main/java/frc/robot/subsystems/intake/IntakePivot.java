package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final SmartMotorController pivotMotor;
  private final Arm pivotArm;

  public IntakePivot() {
    SmartMotorControllerConfig pivotCfg = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.6, 0.0, 0.05, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
        .withFeedforward(new ArmFeedforward(0.1, 0.4, 0.01))
        .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE);

    TalonFX pivotTalonFX = new TalonFX(1);
    pivotMotor = new TalonFXWrapper(pivotTalonFX, DCMotor.getKrakenX60(1), pivotCfg);

    ArmConfig armCfg = new ArmConfig(pivotMotor)
        .withSoftLimits(Degrees.of(-20), Degrees.of(150))
        .withStartingPosition(Degrees.of(0))
        .withLength(Inches.of(30.5))
        .withMass(Pounds.of(4.0))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

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
