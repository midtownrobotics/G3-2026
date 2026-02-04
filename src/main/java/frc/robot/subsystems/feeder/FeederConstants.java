package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public final class FeederConstants {
    public static final Distance kFuelSensorTriggerDistance = Inches.of(5);
    public static final Time kFuelSensorTriggerDebounce = Milliseconds.of(30);
}
