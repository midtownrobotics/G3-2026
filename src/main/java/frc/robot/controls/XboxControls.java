package frc.robot.controls;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

public class XboxControls implements Controls {
  private final IOProtectionXboxController m_controller;
  public XboxControls(int controllerPort) {
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
    return m_controller.rightBumper().and(disableShooting().negate());
  }

  @Override
  public Trigger snowBlow() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger unjam() {
    return m_controller.y().and(fixedShooter().negate());
  }

  @Override
  public Trigger feedFuel() {
    return m_controller.b().and(zeroHood().negate());
  }

  @Override
  public Trigger setpointShoot() {
    return m_controller.a().and(defense().negate());
  }

  @Override
  public Trigger setpointFeed() {
    return m_controller.x().and(zeroIntake().negate());
  }

  @Override
  public Trigger defense() {
    return m_controller.leftBumper().and(m_controller.a());
  }

  @Override
  public Trigger zeroIntake() {
    return m_controller.leftBumper().and(m_controller.x());
  }

  @Override
  public Trigger zeroHood() {
    return m_controller.leftBumper().and(m_controller.b());
  }

  @Override
  public Trigger fixedShooter() {
    return m_controller.leftBumper().and(m_controller.y());
  }

  @Override
  public Trigger disableShooting() {
    return m_controller.leftBumper().and(m_controller.rightBumper());
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
  public Trigger increaseTurretAngle() {
    return m_controller.povRight();
  }

  @Override
  public Trigger decreaseTurretAngle() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger toggleShootOnTheMove() {
    return m_controller.start();
  }

	public Trigger wallAndBulldoze() {
		return m_controller.rightStick();
	}

  public void setRumble(boolean enabled) {
    m_controller.setRumble(RumbleType.kBothRumble, enabled ? 0.5 : 0);
  }

  public Command rumbleCommand() {
    return Commands.run(() -> setRumble(true)).finallyDo(() -> setRumble(false));
  }

  public Command pulseRumbleCommand(int pulses, double pulseDuration) {
    List<Command> commands = new ArrayList<>();

    for (int i = 0; i < pulses; i++) {
      commands.add(rumbleCommand().withTimeout(pulseDuration));

      if (i < pulses - 1) {
        commands.add(Commands.waitSeconds(0.1));
      }
    }

    return Commands.sequence(commands.toArray(Command[]::new));
  }

  @Override
  public void perodic() {
  }
}
