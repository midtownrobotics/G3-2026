
package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.Controls;
import frc.robot.controls.XBoxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RobotViz;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final RobotState m_state;
  private final RobotViz m_viz;

  public Robot() {
    DataLogManager.start();
    Epilogue.bind(this);

    Controls controls = new XBoxControls(0);
    CommandSwerveDrivetrain drive = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,
        TunerConstants.BackLeft,
        TunerConstants.BackRight, TunerConstants.FrontLeft, TunerConstants.FrontRight);
    IntakePivot intakePivot = new IntakePivot();
    IntakeRoller intakeRoller = new IntakeRoller();

    m_state = new RobotState(controls, drive, intakePivot, intakeRoller);
    m_viz = new RobotViz(m_state);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
