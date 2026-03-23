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

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public Angle position = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public Angle encoderAbsolutePosition = Rotations.zero();
    public Angle setpoint = Degrees.zero();
    public boolean motorConnected = false;
  }

  default void updateInputs(HoodIOInputs inputs) {
  }

  default void setPosition(Angle angle) {
  }

  default void setVoltage(Voltage voltage) {
  }

  default void stop() {
  }

  default void setEncoderPosition(Angle angle) {
  }

  default void setLowerSoftLimitEnabled(boolean enabled) {
  }

  default void setPID(double kP, double kI, double kD, double kS, double kG) {
  }
}
