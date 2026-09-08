package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

public class TrimXboxControls implements TrimControls {
  private final IOProtectionXboxController m_controller;

  public TrimXboxControls(int port) {
    m_controller = new IOProtectionXboxController(port);
  }

  private boolean hasController() {
    return DriverStation.getStickAxisCount(1) >= 6
        && DriverStation.getStickButtonCount(1) > 0;
  }

  @Override
  public Trigger increaseFlywheelVelocity() {
    return new Trigger(() -> hasController() && m_controller.getRawAxis(3) > 0.5);
  }

  @Override
  public Trigger decreaseFlywheelVelocity() {
    return new Trigger(() -> hasController() && m_controller.getRawAxis(2) > 0.5);
  }

  @Override
  public Trigger increaseHoodAngle() {
    return new Trigger(() -> hasController() && m_controller.getHID().getPOV() == 0);
  }

  @Override
  public Trigger decreaseHoodAngle() {
    return new Trigger(() -> hasController() && m_controller.getHID().getPOV() == 180);
  }

  @Override
  public Trigger increaseVelocityCompensation() {
    return new Trigger(() -> hasController() && m_controller.getHID().getRawButton(6));
  }

  @Override
  public Trigger decreaseVelocityCompensation() {
    return new Trigger(() -> hasController() && m_controller.getHID().getRawButton(5));
  }

  @Override
  public Trigger increaseTurretAngle() {
    return new Trigger(() -> hasController() && m_controller.getHID().getPOV() == 90);
  }

  @Override
  public Trigger decreaseTurretAngle() {
    return new Trigger(() -> hasController() && m_controller.getHID().getPOV() == 270);
  }
}
