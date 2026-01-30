
package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoAim;
import frc.robot.controls.Controls;
import frc.robot.controls.XboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.turret.Hood;
import frc.robot.subsystems.turret.Shooter;
import frc.robot.subsystems.turret.Turret;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final RobotState m_state;
  private final RobotViz m_viz;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Turret m_turret;
  private final Hood m_hood;
  private final Shooter m_shooter;

  public Robot() {
    DataLogManager.start();
    Epilogue.bind(this);

    m_controls = new XboxControls(0);

    m_drive = TunerConstants.createDrivetrain();

    m_intakePivot = new IntakePivot();
    m_intakeRoller = new IntakeRoller();

    m_turret = new Turret(22);
    m_shooter = new Shooter(23);
    m_hood = new Hood(24);

    m_state = new RobotState(m_controls, m_drive, m_intakePivot, m_intakeRoller, m_turret, m_hood, m_shooter);
    m_viz = new RobotViz(m_state);

    m_turret.setDefaultCommand(AutoAim.getAutoAimCommand(m_state));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_viz.updateViz();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
