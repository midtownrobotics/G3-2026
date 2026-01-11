package frc.robot.utils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class UnitUtil {

  /**
   * Util which returns the max of two measures
   */
  public static <U extends Unit, T extends Measure<U>> T max(T a, T b) {
    if (a.gt(b))
      return a;
    return b;
  }

  /**
   * Util which returns the min of two measures
   */
  public static <U extends Unit, T extends Measure<U>> T min(T a, T b) {
    if (a.lt(b))
      return a;
    return b;
  }

  /**
   * Util which returns the min of two measures
   */
  public static <U extends Unit, T extends Measure<U>> T clamp(T a, T min, T max) {
    return min(max(a, min), max);
  }

  /**
   * Normalizes an angle to be between 0 and 360 degrees
   */
  public static Angle normalize(Angle angle) {
    double degrees = angle.in(Units.Degrees);
    degrees = degrees % 360;
    if (degrees < 0)
      degrees += 360;
    return Units.Degrees.of(degrees);
  }

}
