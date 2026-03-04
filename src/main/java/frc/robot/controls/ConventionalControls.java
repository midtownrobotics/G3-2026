package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents controls with two independent states for intake
 * and shooter. Always use interfaces to interact with controls
 */
@Logged
public interface ConventionalControls {
  public Trigger intake();

  public Trigger shoot();
}
