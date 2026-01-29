package frc.lib;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IOProtectionXboxController extends CommandXboxController {
  public double currentRumbleValue;

  /** Constructs IO protected xbox controller. */
  public IOProtectionXboxController(int port) {
    super(port);
  }

  /** Sets the rumble. */
  public void setRumble(RumbleType rumbleType, double value) {
    if (currentRumbleValue != value) {
      super.setRumble(rumbleType, value);
      currentRumbleValue = value;
    }
  }
}
