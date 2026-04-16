package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RobotCommands;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final RobotCommands m_robotCommands;

  public AutoRoutines(AutoFactory autoFactory, Robot robot, RobotCommands robotCommands) {
    m_autoFactory = autoFactory;
    m_robotCommands = robotCommands;
  }

  public AutoRoutine MadtownLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("MadtownLeft");
    AutoTrajectory TrenchSweep = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory TrenchSweep2 = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BackwardsBump2 = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BumpToTrenchSOTM = routine.trajectory("BumpToTrenchSOTM").mirrorY();

    TrenchSweep.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement());
    TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

    BumpToTrenchSOTM.active().onTrue(m_robotCommands.shootShooterCommand());
    BumpToTrenchSOTM.atTime("stopshoot").onTrue(m_robotCommands.stopShooterCommand());
    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    BackwardsBump2.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            TrenchSweep.resetOdometry(),
            TrenchSweep.cmd()));
    return routine;
  }

  public AutoRoutine MadtownRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("MadtownRight");
    AutoTrajectory TrenchSweep = routine.trajectory("TrenchSweep");
    AutoTrajectory TrenchSweep2 = routine.trajectory("TrenchSweep");
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump");
    AutoTrajectory BackwardsBump2 = routine.trajectory("BackwardsBump");
    AutoTrajectory BumpToTrenchSOTM = routine.trajectory("BumpToTrenchSOTM");

    TrenchSweep.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement());
    TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

    BumpToTrenchSOTM.active().onTrue(m_robotCommands.shootShooterCommand());
    BumpToTrenchSOTM.atTime("stopshoot").onTrue(m_robotCommands.stopShooterCommand());
    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    BackwardsBump2.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            TrenchSweep.resetOdometry(),
            TrenchSweep.cmd()));
    return routine;
  }

  public AutoRoutine HubSwipeLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("HubSwipeLeft");
    AutoTrajectory HubSwipe = routine.trajectory("HubSwipe").mirrorY();

    HubSwipe.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
    HubSwipe.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    HubSwipe.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            HubSwipe.resetOdometry(),
            HubSwipe.cmd()));
    return routine;
  }

  public AutoRoutine HubSwipeRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("HubSwipeRight");
    AutoTrajectory HubSwipe = routine.trajectory("HubSwipe");

    HubSwipe.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
    HubSwipe.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    HubSwipe.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            HubSwipe.resetOdometry(),
            HubSwipe.cmd()));
    return routine;

  }
}
