package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;

  public AutoRoutines(AutoFactory autoFactory) {
    m_autoFactory = autoFactory;
  }

  public AutoRoutine taxiAuto() {
    AutoRoutine routine = m_autoFactory.newRoutine("taxi");

    AutoTrajectory driveToMiddle = routine.trajectory("driveToMiddle");

    routine.active().onTrue(
        Commands.sequence(
            driveToMiddle.resetOdometry(),
            driveToMiddle.cmd()));

    return routine;
  }
}
