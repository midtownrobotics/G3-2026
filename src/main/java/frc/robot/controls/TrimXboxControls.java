package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

public class TrimXboxControls implements TrimControls {
  private final IOProtectionXboxController m_controller;

  public TrimXboxControls(int port) {
    m_controller = new IOProtectionXboxController(port);
  }

  @Override
  public Trigger increaseFlywheelVelocity() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger decreaseFlywheelVelocity() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger increaseHoodAngle() {
    return m_controller.povUp();
  }

  @Override
  public Trigger decreaseHoodAngle() {
    return m_controller.povDown();
  }

  @Override
  public Trigger increaseVelocityCompensation() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger decreaseVelocityCompensation() {
    return m_controller.leftBumper();
  }

}
