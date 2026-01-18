package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface Controls {
  public static double DRIVER_JOYSTICK_THRESHOLD = 0.1;

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();
}
