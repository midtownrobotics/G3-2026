package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Used for in-match maintainance and fall backs. Normally,
 * something like this would be an elastic button, but
 * since we only have one driver, a trim controller allows us
 * to have a second driver who can perform in-match maintainance
 * more quickly
 */
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
