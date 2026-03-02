package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

public class FourWayXboxControls implements FourWayControls, DriveControls {
  private final IOProtectionXboxController m_controller;

  public FourWayXboxControls(int controllerPort) {
    m_controller = new IOProtectionXboxController(controllerPort);
  }

  @Override
  public double getDriveForward() {
    return MathUtil.applyDeadband(m_controller.getLeftY() * -1, kDriverJoystickThreshold);
  }

  @Override
  public double getDriveLeft() {
    return MathUtil.applyDeadband(m_controller.getLeftX() * -1, kDriverJoystickThreshold);
  }

  @Override
  public double getDriveRotation() {
    return MathUtil.applyDeadband(m_controller.getRightX() * -1, kDriverJoystickThreshold);
  }

  @Override
  public Trigger idle() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger intake() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger shoot() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger snowBlow() {
    return m_controller.rightTrigger();
  }
}
