package frc.robot.subsystems.superstructure.constraints;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import lombok.EqualsAndHashCode;
import lombok.Getter;

@EqualsAndHashCode
public class Interval<U extends Unit, T extends Measure<U>> {
  @Getter
  private T start;
  @Getter
  private T end;

  public Interval(T start, T end) {
    if (start.gt(end))
      throw new IllegalArgumentException("Start cannot be greater than end");
  }

  public boolean contains(T value) {
    return value.gte(start) && value.lte(end);
  }

  @Override
  public String toString() {
    if (start instanceof Angle startAngle && end instanceof Angle endAngle) {
      return "[" + startAngle.in(Degrees) + "," + endAngle.in(Degrees) + "]";
    }
    return "[" + start.baseUnitMagnitude() + "," + end.baseUnitMagnitude() + "]";
  }
}
