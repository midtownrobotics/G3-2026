package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public interface TrimControls {
  public Trigger increaseFlywheelVelocity();

  public Trigger decreaseFlywheelVelocity();

  public Trigger increaseHoodAngle();

  public Trigger decreaseHoodAngle();

  public Trigger increaseVelocityCompensation();

  public Trigger decreaseVelocityCompensation();

  public Trigger increaseTurretAngle();

  public Trigger decreaseTurretAngle();
}
