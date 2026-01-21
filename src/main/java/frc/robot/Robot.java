package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.controls.Controls;
import frc.robot.controls.XboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final RobotState m_state;
  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

  public Robot() {
    DataLogManager.start();
    Epilogue.bind(this);

    m_controls = new XboxControls(0);
    m_drive = TunerConstants.createDrivetrain();

    m_state = new RobotState(m_controls, m_drive);
  
    m_autoFactory = m_drive.createAutoFactory();
    m_autoRoutines = new AutoRoutines(m_autoFactory);
    m_autoChooser = new AutoChooser("Do Nothing");
    generateAutoChooser();
  }

  private void generateAutoChooser() {
    m_autoChooser.addRoutine("Taxi", m_autoRoutines::taxiAuto);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());

    // RobotModeTriggers.autonomous().whileTrue(m_autoRoutines.taxiAuto().cmd());
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
