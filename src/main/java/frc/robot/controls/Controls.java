package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {
  public static double kDriverJoystickThreshold = 0.1;

  public Trigger idle();

  public Trigger intake();

  public Trigger shoot();

  public Trigger defense();

  public Trigger snowBlow();

  public Trigger unjam();

  public Trigger zeroHood();

  public Trigger zeroIntake();

  public Trigger setpointShoot();

  public Trigger setpointFeed();

  public Trigger feedFuel();

  public Trigger fixedShooter();

  public Trigger disableShooting();

  public Trigger increaseHoodAngle();

  public Trigger decreaseHoodAngle();

  public Trigger increaseTurretAngle();

  public Trigger decreaseTurretAngle();

	public Trigger wallAndBulldoze();

  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Command rumbleCommand();

  public Command pulseRumbleCommand(int pulses, double pulseDuration);

  public Trigger toggleShootOnTheMove();

  public void perodic();
}
