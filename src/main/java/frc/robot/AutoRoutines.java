package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;

  public AutoRoutines(AutoFactory autoFactory) {
    m_autoFactory = autoFactory;
  }

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");
    
    routine.active().onTrue(
      Commands.sequence(
        depotToMidLeft.resetOdometry(),
        depotToMidLeft.cmd()));
    return routine;
  }

  public AutoRoutine leftStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("leftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("leftStartToCenter");
    
    routine.active().onTrue(
      Commands.sequence(
        leftStartToCenter.resetOdometry(),
        leftStartToCenter.cmd()));
    return routine;
  }

}

