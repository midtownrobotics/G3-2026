package frc.robot.controls;

/**
 * Shared interface between all control types
 * alows drive control bindings to be set independently
 * of the specific current control type
 */
public interface DriveControls {
  public static double kDriverJoystickThreshold = 0.1;

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();
}
