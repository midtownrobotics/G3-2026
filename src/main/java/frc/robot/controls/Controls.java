package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public interface Controls {
  public static double kDriverJoystickThreshold = 0.1;

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger getIntake();
}
