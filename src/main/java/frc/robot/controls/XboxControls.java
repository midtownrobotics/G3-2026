package frc.robot.controls;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;

public class XboxControls implements Controls {
  private final IOProtectionXboxController m_controller;
  private boolean m_controlsLocked = false;

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
    return m_controller.leftBumper().and(defense().negate()).and(fixedShooter().negate()).and(zeroHood().negate())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger intake() {
    return m_controller.leftTrigger().and(defense().negate()).and(fixedShooter().negate()).and(zeroIntake().negate())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger shoot() {
    return m_controller.rightBumper().and(disableShooting().negate()).and(fixedShooter().negate())
        .and(zeroHood().negate()).and(controlsLocked().negate());
  }

  @Override
  public Trigger snowBlow() {
    return m_controller.rightTrigger().and(disableShooting().negate()).and(fixedShooter().negate())
        .and(zeroIntake().negate()).and(controlsLocked().negate());
  }

  @Override
  public Trigger unjam() {
    return m_controller.y().and(controlsLocked().negate());
  }

  @Override
  public Trigger feedFuel() {
    return m_controller.b().and(controlsLocked().negate());
  }

  @Override
  public Trigger setpointShoot() {
    return m_controller.a().and(controlsLocked().negate());
  }

  @Override
  public Trigger setpointFeed() {
    return m_controller.x().and(controlsLocked().negate());
  }

  @Override
  public Trigger defense() {
    return m_controller.leftTrigger().and(m_controller.leftBumper())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger zeroIntake() {
    return m_controller.leftTrigger().and(m_controller.rightTrigger())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger zeroHood() {
    return m_controller.leftBumper().and(m_controller.rightBumper())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger fixedShooter() {
    return m_controller.leftTrigger()
        .and(m_controller.leftBumper())
        .and(m_controller.rightTrigger())
        .and(m_controller.rightBumper())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger disableShooting() {
    return m_controller.rightTrigger().and(m_controller.rightBumper())
        .and(controlsLocked().negate());
  }

  @Override
  public Trigger increaseHoodAngle() {
    return m_controller.povUp().and(controlsLocked().negate());
  }

  @Override
  public Trigger decreaseHoodAngle() {
    return m_controller.povDown().and(controlsLocked().negate());
  }

  @Override
  public Trigger increaseTurretAngle() {
    return m_controller.povRight().and(controlsLocked().negate());
  }

  @Override
  public Trigger decreaseTurretAngle() {
    return m_controller.povLeft().and(controlsLocked().negate());
  }

  @Override
  public Trigger toggleShootOnTheMove() {
    return m_controller.start().and(controlsLocked().negate());
  }

  @Override
  public Trigger controlsLocked() {
    return new Trigger(() -> m_controlsLocked);
  }

  @Override
  public Command comboCommand(Trigger heldTrigger, Command action) {
    Timer confirmTimer = new Timer();
    return Commands.sequence(
        Commands.runOnce(() -> {
          confirmTimer.restart();
          setRumble(true);
        }),
        Commands.waitSeconds(1.5).until(heldTrigger.negate()),
        Commands.runOnce(() -> setRumble(false)),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> m_controlsLocked = true),
                Commands.parallel(action,
                    Commands.waitSeconds(1.5).andThen(Commands.runOnce(() -> m_controlsLocked = false)))),
            Commands.none(),
            () -> confirmTimer.hasElapsed(1.5)))
        .finallyDo(() -> {
          setRumble(false);
          m_controlsLocked = false;
        });
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
    Logger.recordOutput("Controls/controlsLocked", m_controlsLocked);
  }
}
