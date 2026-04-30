package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public AngularVelocity velocity1 = RPM.zero();
		public AngularVelocity velocity2 = RPM.zero();
    public Voltage appliedVoltage1 = Volts.zero();
		public Voltage appliedVoltage2 = Volts.zero();
    public Current statorCurrent1 = Amps.zero();
    public Current supplyCurrent1 = Amps.zero();
		public Current statorCurrent2 = Amps.zero();
    public Current supplyCurrent2 = Amps.zero();
    public Distance fuelSensorDistance = Meters.of(Double.MAX_VALUE);
    public AngularVelocity setpoint = RPM.zero();
    public boolean motorConnected1 = false;
		public boolean motorConnected2 = false;
  }

  default void updateInputs(FeederIOInputs inputs) {
  }

  default void setSpeed(AngularVelocity speed) {
  }

  default void setVoltage(Voltage voltage) {
  }

  default void stop() {
  }

  default void setPID(double kP, double kI, double kD) {
  }
}
