package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public interface ConventionalControls {
  public Trigger intake();

  public Trigger shoot();
}
