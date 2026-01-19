package frc.robot.controls;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

@Logged
public class XBoxControls implements Controls {
  private final IOProtectionXboxController m_controller;

  public XBoxControls(int controllerPort) {
    m_controller = new IOProtectionXboxController(controllerPort);
  }

  public double getDriveForward() {
    return MathUtil.applyDeadband(m_controller.getLeftY(), DRIVER_JOYSTICK_THRESHOLD);
  }

  public double getDriveLeft() {
    return MathUtil.applyDeadband(m_controller.getLeftX(), DRIVER_JOYSTICK_THRESHOLD);

  }

  public double getDriveRotation() {
    return MathUtil.applyDeadband(m_controller.getRightX(), DRIVER_JOYSTICK_THRESHOLD);
  }

  public Trigger intake() {
    return m_controller.a();
  }
}
