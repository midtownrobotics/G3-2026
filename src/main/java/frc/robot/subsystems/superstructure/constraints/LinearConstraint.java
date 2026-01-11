package frc.robot.subsystems.superstructure.constraints;

import static frc.robot.utils.UnitUtil.clamp;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class LinearConstraint<U extends Unit, M extends Measure<U>> {
  private M lower;
  private M upper;

  /**
   * Constructor for a new Linear Constraint
   */
  public LinearConstraint(M lower, M upper) {
    this.lower = lower;
    this.upper = upper;
  }

  /**
   * Clamps the value to the range of the constraint
   */
  public M getClampedValue(M value) {
    return clamp(value, lower, upper);
  }

  /**
   * Sets the lower and upper bounds to the same value
   */
  public void restrictToValue(M value) {
    lower = value;
    upper = value;
  }
}
