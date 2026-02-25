package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface FourWayControls {
  public Trigger idle();

  public Trigger fill();

  public Trigger shoot();

  public Trigger snowBlow();
}
