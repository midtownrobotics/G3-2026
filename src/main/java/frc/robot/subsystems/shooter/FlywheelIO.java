package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public AngularVelocity velocity = RPM.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent1 = Amps.zero();
    public Current statorCurrent2 = Amps.zero();
    public Current supplyCurrent1 = Amps.zero();
    public Current supplyCurrent2 = Amps.zero();
    public Voltage feedForwardVoltage = Volts.zero();
    public AngularVelocity velocity1 = RotationsPerSecond.zero();
    public AngularVelocity velocity2 = RotationsPerSecond.zero();
    public AngularVelocity setpoint = RPM.zero();
    public boolean motor1Connected = false;
    public boolean motor2Connected = false;
  }

  default void updateInputs(FlywheelIOInputs inputs) {
  }

  default void setSpeed(AngularVelocity speed) {
  }

  default void setSpeed(AngularVelocity speed, Voltage feedForward) {
  }

  default void setVoltage(Voltage voltage) {
  }

  default void bangBang(Voltage voltage, AngularVelocity targetSpeed) {
  }

  default void stop() {
  }

  default void setPID(double kP, double kI, double kD, double kS, double kV) {
  }
}
