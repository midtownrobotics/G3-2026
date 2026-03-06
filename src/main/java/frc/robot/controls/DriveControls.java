package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface DriveControls {
  public static double kDriverJoystickThreshold = 0.1;

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();
}
