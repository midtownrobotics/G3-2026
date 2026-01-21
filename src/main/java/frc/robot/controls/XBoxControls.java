package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

@Logged
public class XboxControls implements Controls {
  private final IOProtectionXboxController m_controller;

  public XboxControls(int controllerPort) {
    m_controller = new IOProtectionXboxController(controllerPort);
  }

  public double getDriveForward() {
    return MathUtil.applyDeadband(m_controller.getLeftY() * -1, kDriverJoystickThreshold);
  }

  public double getDriveLeft() {
    return MathUtil.applyDeadband(m_controller.getLeftX() * -1, kDriverJoystickThreshold);

  }

  public double getDriveRotation() {
    return MathUtil.applyDeadband(m_controller.getRightX() * -1, kDriverJoystickThreshold);

  }

  @Override
  public Trigger intake() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger shoot() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger autoIntake() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger switchView() {
    return m_controller.leftBumper();
  }
}
