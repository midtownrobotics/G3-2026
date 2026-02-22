package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public enum IntakeSetpoint {
  STOW(Degrees.of(90), Volts.of(0)),
  INTAKING(Degrees.of(0), Volts.of(7));

  public final Angle angle;
  public final Voltage voltage;

  IntakeSetpoint(Angle angle, Voltage voltage) {
    this.angle = angle;
    this.voltage = voltage;
  }
}
