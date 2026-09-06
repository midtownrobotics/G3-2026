package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Gains;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public AngularVelocity velocity = RPM.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent1 = Amps.zero();
    public Current statorCurrent2 = Amps.zero();
    public Current supplyCurrent1 = Amps.zero();
    public Current supplyCurrent2 = Amps.zero();
    /** Signed torque-producing current. This is what the closed loop commands under torque FOC. */
    public Current torqueCurrent1 = Amps.zero();
    public Current torqueCurrent2 = Amps.zero();
    public AngularVelocity velocity1 = RotationsPerSecond.zero();
    public AngularVelocity velocity2 = RotationsPerSecond.zero();
    public AngularVelocity setpoint = RPM.zero();
    public AngularVelocity closedLoopReference = RPM.zero();
    public AngularVelocity closedLoopError = RPM.zero();
    public boolean motor1Connected = false;
    public boolean motor2Connected = false;
  }

  default void updateInputs(FlywheelIOInputs inputs) {
  }

  default void setSpeed(AngularVelocity speed) {
  }

  default void setVoltage(Voltage voltage) {
  }

  /** Open-loop torque-current control, used to characterize kS and kA. */
  default void setTorqueCurrent(Current current) {
  }

  default void bangBang(Voltage voltage, AngularVelocity targetSpeed) {
  }

  default void stop() {
  }

  default void setGains(Gains gains) {
  }
}
