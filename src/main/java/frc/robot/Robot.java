package frc.robot;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.Controls;
import frc.robot.controls.XboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final RobotState m_state;

  private final IntakePivot m_intakePivot = new IntakePivot();
  private final IntakeRoller m_intakeRoller = new IntakeRoller();

  public Robot() {
    DataLogManager.start();
    Epilogue.bind(this);

    m_controls = new XboxControls(0);
    m_drive = TunerConstants.createDrivetrain();

    m_state = new RobotState(m_controls, m_drive);

    m_controls.getButtonA().whileTrue(intakeCommand());
  }

  private Command intakeCommand() {
    return new Command() {
      public void initialize() {
        m_intakePivot.setAngleCommand(IntakeGoal.INTAKING.angle).schedule();
        m_intakeRoller.setSpeedCommand(IntakeGoal.INTAKING.rollerDutyCycle.in(Percent)).schedule();
      }

      public void end(boolean interrupted) {
        m_intakePivot.setAngleCommand(IntakeGoal.STOW.angle).schedule();
        m_intakeRoller.setSpeedCommand(IntakeGoal.STOW.rollerDutyCycle.in(Percent)).schedule();
      }
    };
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
