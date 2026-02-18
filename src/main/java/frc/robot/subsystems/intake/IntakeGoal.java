package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public enum IntakeGoal {
  STOW(Degrees.of(70), Volts.of(0)),
  INTAKING(Degrees.of(5), Volts.of(7));

  public final Angle angle;
  public final Voltage voltage;

  IntakeGoal(Angle angle, Voltage voltage) {
    this.angle = angle;
    this.voltage = voltage;
  }
}
