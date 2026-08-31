package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.Gains;
import frc.lib.MotionProfile;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Angle position = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    /** Signed torque-producing current. This is what the closed loop commands under torque FOC. */
    public Current torqueCurrent = Amps.zero();
    public Angle encoder1AbsolutePosition = Rotations.zero();
    public Angle encoder2AbsolutePosition = Rotations.zero();
    public Angle setpoint = Degrees.zero();
    /** Motion Magic's current profiled target, which lags {@link #setpoint} during a move. */
    public Angle closedLoopReference = Degrees.zero();
    public Angle closedLoopError = Degrees.zero();
    public boolean motorConnected = false;
  }

  default void updateInputs(TurretIOInputs inputs) {
  }

  default void setPosition(Angle angle) {
  }

  default void setVoltage(Voltage voltage) {
  }

  /** Open-loop torque-current control, used to characterize kS and kA. */
  default void setTorqueCurrent(Current current) {
  }

  default void stop() {
  }

  default void setEncoderPosition(Angle angle) {
  }

  default void setGains(Gains gains) {
  }

  default void setMotionProfile(MotionProfile profile) {
  }
}
