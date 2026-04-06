package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public Angle position = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public Angle encoderAbsolutePosition = Degrees.zero();
    public Angle setpoint = Degrees.zero();
    public boolean motorConnected = false;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {
  }

  default void setPosition(Angle angle) {
  }

  default void setVoltage(Voltage voltage) {
  }

  default void stop() {
  }

  default void setEncoderPosition(Angle angle) {
  }

  default void setPID(double kP, double kI, double kD, double kG) {
  }

  default void setLowerSoftLimitEnabled(boolean enabled) {
  }
}
