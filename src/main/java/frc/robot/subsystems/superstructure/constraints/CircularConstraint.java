package frc.robot.subsystems.superstructure.constraints;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.UnitUtil;
import lombok.NonNull;

public class CircularConstraint {

  public RealNumberSet<AngleUnit, Angle> intervals;

  /**
   * Constructor for a new Circular Constraint
   */
  public CircularConstraint() {
    intervals = new RealNumberSet<>();
    intervals.add(new Interval<>(Degrees.of(0), Degrees.of(360)));
  }

  /**
   * Adds a constraint that the final angle must be between two angles
   */
  public CircularConstraint addStayInConstraint(Angle start, Angle end) {
    start = UnitUtil.normalize(start);
    end = UnitUtil.normalize(end);

    RealNumberSet<AngleUnit, Angle> intersectedSet = new RealNumberSet<>();
    if (end.gt(start)) {
      intersectedSet.add(new Interval<>(start, end));
    } else {
      intersectedSet.add(new Interval<>(Degrees.of(0), end));
      intersectedSet.add(new Interval<>(start, Degrees.of(360)));
    }

    intervals = intervals.intersection(intersectedSet);

    return this;
  }

  /**
   * Adds a constraint that the final angle must be outside of two angles
   */
  public CircularConstraint addKeepOutConstraint(Angle start, Angle end) {
    start = UnitUtil.normalize(start);
    end = UnitUtil.normalize(end);

    RealNumberSet<AngleUnit, Angle> differenceSet = new RealNumberSet<>();
    if (end.gte(start)) {
      differenceSet.add(new Interval<>(start, end));
    } else {
      differenceSet.add(new Interval<>(Degrees.of(0), end));
      differenceSet.add(new Interval<>(start, Degrees.of(360)));
    }

    intervals = intervals.difference(differenceSet);

    return this;
  }

  /**
   * Intersects two constraints together
   */
  public CircularConstraint addConstraint(CircularConstraint constraint) {
    intervals = intervals.intersection(constraint.intervals);
    return this;
  }

  /**
   * Checks if an angle is valid to reach under the current constraints.
   */
  public boolean isValid(Angle angle) {
    angle = UnitUtil.normalize(angle);
    Interval<AngleUnit, Angle> interval = unwrapInterval(intervals.getIntervalOfValue(angle));

    if (interval == null)
      return false;

    return interval.contains(angle) || interval.contains(angle.plus(Degrees.of(360)))
        || interval.contains(angle.minus(Degrees.of(360)));
  }

  /**
   * Returns a wraparound interval (from -360 to 720) of the given interval.
   * For example, if current constraint is [[0, 50], [300, 360]]:
   * getWraparoundInterval([0, 50]) --> [-60, 50]
   * getWraparoundInterval([300, 360]]) --> [300, 410]
   */
  private Interval<AngleUnit, Angle> unwrapInterval(Interval<AngleUnit, Angle> interval) {
    if (interval == null)
      return null;

    Angle start = interval.getStart();
    Angle end = interval.getEnd();

    if (interval.contains(Degrees.of(360))) {
      Interval<AngleUnit, Angle> wrapAround = intervals.getIntervalOfValue(Degrees.of(0));
      if (wrapAround != null) {
        end = wrapAround.getEnd().plus(Degrees.of(360));
      }
    }

    if (interval.contains(Degrees.of(0))) {
      Interval<AngleUnit, Angle> wrapAround = intervals.getIntervalOfValue(Degrees.of(360));
      if (wrapAround != null) {
        start = wrapAround.getStart().minus(Degrees.of(360));
      }
    }

    return new Interval<AngleUnit, Angle>(start, end);
  }

  /**
   * Gets the delta to be as close to the desired angle as possible
   * @param current A normalized angle of the current position of the system
   * @param desired A normalized angle of the desired position of the system
   * @return Best delta to target based on constraints. Null if the current position is impossible.
   */
  private @NonNull Angle getDeltaToDesired(Angle current, Angle desired) {
    // Get the wraparound Interval of the current positive
    Interval<AngleUnit, Angle> interval = unwrapInterval(intervals.getIntervalOfValue(current));

    if (interval == null) {
      /** Set complement gives intervals of illegal values rather than legal ones */
      RealNumberSet<AngleUnit, Angle> inverted = intervals.complement(Degrees.of(0), Degrees.of(360));
      Interval<AngleUnit, Angle> invertedInterval = unwrapInterval(inverted.getIntervalOfValue(current));

      if (invertedInterval.getStart().isEquivalent(Degrees.of(-360))
          && invertedInterval.getEnd().isEquivalent(Degrees.of(720))) {
        return current;
      }

      /** These are two possible legal options, the start or the end. */
      Angle previousLegal = invertedInterval.getStart();
      Angle nextLegal = invertedInterval.getEnd();

      // If closer to previous legal, return that. Else return next legal.
      if (previousLegal.minus(current).abs(current.baseUnit()) < nextLegal.minus(current).abs(current.baseUnit())) {
        return previousLegal.minus(current);
      }

      return nextLegal.minus(current);
    }
    ;

    // Find the goal in the positive direction
    Angle positiveGoal = desired;
    if (desired.lt(current)) {
      positiveGoal = desired.plus(Degrees.of(360));
    }

    // Find how close we can get to that goal
    Angle positiveOutput = min(positiveGoal, interval.getEnd());

    // Find the goal in the negative direction
    Angle negativeGoal = desired;
    if (desired.gt(current)) {
      negativeGoal = desired.minus(Degrees.of(360));
    }

    // Find how close we can get to that goal
    Angle negativeOutput = max(negativeGoal, interval.getStart());

    // Calculate the delta travelled in each direction
    Angle positiveDelta = positiveOutput.minus(current);
    Angle negativeDelta = negativeOutput.minus(current);

    // Choose positive if its closer to goal
    if (negativeOutput.minus(negativeGoal).abs(Degrees) > positiveOutput.minus(positiveGoal).abs(Degrees)) {
      return positiveDelta;
    }

    // Choose negative if its closer to goal
    if (negativeOutput.minus(negativeGoal).abs(Degrees) < positiveOutput.minus(positiveGoal).abs(Degrees)) {
      return negativeDelta;
    }

    // If both are equidistance, chose whichever is closer to current position
    if (positiveDelta.abs(Degrees) < negativeDelta.abs(Degrees)) {
      return positiveDelta;
    }

    return negativeDelta;
  }

  /**
   * Gets the angle closest to the desired angle based on the current angle and these constraints.
   * @param current A non-normalized angle of the current position of the system
   * @param desired A normalized angle (can be not normalized) of the desired position of the system
   * @return Best angle to target based on constraints. Null if the current position is impossible.
   */
  public Angle getClosestToDesired(Angle current, Angle desired) {
    Angle delta = getDeltaToDesired(UnitUtil.normalize(current), UnitUtil.normalize(desired));

    return current.plus(delta);
  }

  @Override
  public String toString() {
    return intervals.toString();
  }
}
