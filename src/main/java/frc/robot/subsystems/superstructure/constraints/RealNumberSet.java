package frc.robot.subsystems.superstructure.constraints;

import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;

public class RealNumberSet<U extends Unit, T extends Measure<U>> {
  @Getter
  protected List<Interval<U, T>> intervals;

  public RealNumberSet() {
    this.intervals = new ArrayList<>();
  }

  public RealNumberSet<U, T> add(Interval<U, T> interval) {
    int i = 0;
    T start = interval.getStart();
    T end = interval.getEnd();

    while (i < intervals.size() && intervals.get(i).getEnd().lt(start)) {
      i++;
    }

    while (i < intervals.size() && intervals.get(i).getStart().lte(end)) {
      Interval<U, T> removedInterval = intervals.remove(i);
      start = min(start, removedInterval.getStart());
      end = max(end, removedInterval.getEnd());
    }

    intervals.add(i, new Interval<U, T>(start, end));

    return this;
  }

  public RealNumberSet<U, T> union(RealNumberSet<U, T> other) {
    RealNumberSet<U, T> result = new RealNumberSet<U, T>();

    int i = 0, j = 0;

    while (i < this.intervals.size() || j < other.intervals.size()) {
      Interval<U, T> next;
      if (j >= other.intervals.size()
          || (i < this.intervals.size() && this.intervals.get(i).getStart().lt(other.intervals.get(j).getEnd()))) {
        next = this.intervals.get(i++);
      } else {
        next = other.intervals.get(j++);
      }
      result.add(new Interval<U, T>(next.getStart(), next.getEnd()));
    }
    return result;
  }

  public RealNumberSet<U, T> intersection(RealNumberSet<U, T> other) {
    RealNumberSet<U, T> result = new RealNumberSet<U, T>();
    int i = 0, j = 0;
    while (i < this.intervals.size() && j < other.intervals.size()) {
      Interval<U, T> a = this.intervals.get(i);
      Interval<U, T> b = other.intervals.get(j);

      // Find overlap
      T start = max(a.getStart(), b.getStart());
      T end = min(a.getEnd(), b.getEnd());

      if (start.lte(end)) { // Overlapping
        result.add(new Interval<>(start, end));
      }

      // Move to the next interval
      if (a.getEnd().lt(b.getEnd())) {
        i++;
      } else {
        j++;
      }
    }
    return result;
  }

  public RealNumberSet<U, T> difference(RealNumberSet<U, T> other) {
    RealNumberSet<U, T> result = new RealNumberSet<U, T>();
    for (Interval<U, T> a : this.intervals) {
      T currentStart = a.getStart();
      T currentEnd = a.getEnd();
      for (Interval<U, T> b : other.intervals) {
        if (b.getEnd().lte(currentStart))
          continue;
        if (b.getStart().gte(currentEnd))
          break;
        if (b.getStart().gt(currentStart)) {
          result.add(new Interval<>(currentStart, min(b.getStart(), currentEnd)));
        }
        currentStart = max(currentStart, b.getEnd());
        if (currentStart.gte(currentEnd))
          break;
      }
      if (currentStart.lt(currentEnd)) {
        result.add(new Interval<>(currentStart, currentEnd));
      }
    }
    return result;
  }

  public RealNumberSet<U, T> complement(T lowerBound, T upperBound) {
    RealNumberSet<U, T> result = new RealNumberSet<>();
    T currentStart = lowerBound;

    for (Interval<U, T> interval : intervals) {
      T intervalStart = interval.getStart();
      T intervalEnd = interval.getEnd();

      if (currentStart.gte(upperBound))
        break;

      if (currentStart.lt(intervalStart)) {
        result.add(new Interval<>(currentStart, intervalStart));
      }
      currentStart = max(currentStart, intervalEnd);
    }

    if (currentStart.lt(upperBound)) {
      result.add(new Interval<>(currentStart, upperBound));
    }

    return result;
  }

  /**
   * Returns the interval that contains the given value
   */
  public Interval<U, T> getIntervalOfValue(T value) {
    for (Interval<U, T> interval : intervals) {
      if (interval.contains(value))
        return interval;
    }
    return null;
  }

  @Override
  public String toString() {
    return intervals.toString();
  }
}
