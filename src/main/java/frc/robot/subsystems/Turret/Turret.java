package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
  private final TalonFX m_yawMotor;
  private final Pivot m_yawPivot;
  private final CANcoder m_yawCANCoder1;
  private final CANcoder m_yawCANCoder2;

  public Turret(int yawMotorID, int yawMotorEncoderID1, int yawMotorEncoderID2) {
    m_yawMotor = new TalonFX(yawMotorID);
    m_yawCANCoder1 = new CANcoder(yawMotorEncoderID1);
    m_yawCANCoder2 = new CANcoder(yawMotorEncoderID2);

    SmartMotorControllerConfig yawMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(TurretConstants.kYawP, TurretConstants.kYawI, TurretConstants.kYawD,
            TurretConstants.kYawMotorMaxAngularVelocity, DegreesPerSecondPerSecond.of(30))
        .withGearing(TurretConstants.kYawGearReduction)
        .withIdleMode(MotorMode.BRAKE)
        .withTelemetry("Yaw Motor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(TurretConstants.kMotorCurrentLImit)
        .withClosedLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate))
        .withOpenLoopRampRate(Seconds.of(TurretConstants.kYawPIDRampRate));

    SmartMotorController yawMotorController = new TalonFXWrapper(m_yawMotor, DCMotor.getKrakenX60(1), yawMotorConfig);

    PivotConfig yawMotorPivotConfig = new PivotConfig(yawMotorController)
        .withStartingPosition(chineseRemainderTheorem(
            m_yawCANCoder1.getAbsolutePosition().getValueAsDouble(),
            m_yawCANCoder2.getAbsolutePosition().getValueAsDouble(),
            TurretConstants.kCRTRatio1,
            TurretConstants.kCRTRatio2))
        .withWrapping(Degrees.of(0), Degrees.of(360))
        .withHardLimit(Degrees.of(0), TurretConstants.kYawPivotHardLimit)
        .withTelemetry("Yaw Pivot", TelemetryVerbosity.HIGH)
        .withMOI(TurretConstants.kYawPivotDiameter, TurretConstants.kYawPivotMass);

    m_yawPivot = new Pivot(yawMotorPivotConfig);
  }

  private Angle chineseRemainderTheorem(double encoder1Rotations, double encoder2Rotations, double ratio1,
      double ratio2) {
    double a1 = encoder1Rotations * ratio1;
    double a2 = encoder2Rotations * ratio2;
    double m1 = ratio1;
    double m2 = ratio2;
    double M = m1 * m2;
    double M1 = M / m1;
    double M2 = M / m2;
    double y1 = modInverse(M1, m1);
    double y2 = modInverse(M2, m2);
    double position = (a1 * M1 * y1 + a2 * M2 * y2) % M;
    return Degrees.of((position / M) * 360.0);
  }

  private double modInverse(double a, double m) {
    a = a % m;
    for (int x = 1; x < m; x++) {
      if ((a * x) % m == 1) {
        return x;
      }
    }
    return 1;
  }

  public void periodic() {
  }

  public Angle getYawAngle() {
    return m_yawPivot.getAngle();
  }

  public Command setYawAngleCommand(Angle angle) {
    return m_yawPivot.setAngle(angle);
  }
}
