package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;

public enum IntakeGoal {
  STOW(Degrees.of(90), Percent.of(0)),
  INTAKING(Degrees.of(0), Percent.of(100));

  public final Angle angle;
  public final Dimensionless rollerDutyCycle;

  IntakeGoal(Angle angle, Dimensionless rollerDutyCycle) {
    this.angle = angle;
    this.rollerDutyCycle = rollerDutyCycle;
  }
}
