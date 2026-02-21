package frc.robot.controls;

public interface DriveControls {
  public static double kDriverJoystickThreshold = 0.1;

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();
}
