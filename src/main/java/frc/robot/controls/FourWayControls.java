package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public interface FourWayControls {
  public Trigger idle();

  public Trigger intake();

  public Trigger shoot();

  public Trigger snowBlow();

  public Trigger unjam();

  public Trigger zeroHood();

  public Trigger setpointShoot();
}
