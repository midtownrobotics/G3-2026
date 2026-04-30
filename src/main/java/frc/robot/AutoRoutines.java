package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LoggedTunableNumber;
import frc.robot.commands.RobotCommands;

public class AutoRoutines {

    private final AutoFactory m_autoFactory;
    private final RobotCommands m_robotCommands;
		private final LoggedTunableNumber m_hubSwipeDelaySeconds = new LoggedTunableNumber("HubSwipeDelaySeconds", 0.0);

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
        TrenchSweep.active().onTrue(m_robotCommands.runIntake());

        TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
        // TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        TrenchSweep.done().onTrue(BackwardsBump.cmd());

        BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.active().onTrue(m_robotCommands.shootShooterCommand());
        BumpToTrenchSOTM.atTime(7.5).onTrue(m_robotCommands.stopShooterCommand());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        // TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

        BackwardsBump2.done().onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
												m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(1)),
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

        TrenchSweep.active().onTrue(m_robotCommands.runIntake());
        TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
        // TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        TrenchSweep.done().onTrue(BackwardsBump.cmd());

        BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.active().onTrue(m_robotCommands.shootShooterCommand());
        BumpToTrenchSOTM.atTime(7.5).onTrue(m_robotCommands.stopShooterCommand());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        // TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

        BackwardsBump2.doneDelayed(0.5).onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
												m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(1)),
                        TrenchSweep.resetOdometry(),
                        TrenchSweep.cmd()));
        return routine;
    }

    public AutoRoutine HubSwipeLeft() {
        AutoRoutine routine = m_autoFactory.newRoutine("HubSwipeLeft");
        AutoTrajectory HubSwipe = routine.trajectory("HubSwipe").mirrorY();

        HubSwipe.active().onTrue(m_robotCommands.runIntake().asProxy());
        HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
        HubSwipe.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        HubSwipe.done().onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
												Commands.defer(() ->  Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
                        HubSwipe.resetOdometry(),
                        HubSwipe.cmd()));
        return routine;
    }

    public AutoRoutine HubSwipeRight() {
        AutoRoutine routine = m_autoFactory.newRoutine("HubSwipeRight");
        AutoTrajectory HubSwipe = routine.trajectory("HubSwipe");

        HubSwipe.active().onTrue(m_robotCommands.runIntake().asProxy());
        HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
        HubSwipe.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        HubSwipe.done().onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
												Commands.defer(() ->  Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
                        HubSwipe.resetOdometry(),
                        HubSwipe.cmd()));
        return routine;

    }

    public AutoRoutine copy1002right() {
        AutoRoutine routine = m_autoFactory.newRoutine("1002right");
        AutoTrajectory copy1002left = routine.trajectory("copy1002");
        AutoTrajectory copy1002left2 = routine.trajectory("copy1002");
        AutoTrajectory trenchLineUp1002 = routine.trajectory("trenchLineUp1002");

        copy1002left.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left.done().onTrue(m_robotCommands.shootShooterCommand());
        copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

        trenchLineUp1002.active().onTrue(m_robotCommands.stopShooterCommand());
        trenchLineUp1002.done().onTrue(copy1002left2.cmd());

        copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left2.done().onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
                        copy1002left.resetOdometry(),
                        copy1002left.cmd()));
        return routine;
    }

    public AutoRoutine copy1002left() {
        AutoRoutine routine = m_autoFactory.newRoutine("1002left");
        AutoTrajectory copy1002left = routine.trajectory("copy1002").mirrorY();
        AutoTrajectory copy1002left2 = routine.trajectory("copy1002").mirrorY();
        AutoTrajectory trenchLineUp1002 = routine.trajectory("trenchLineUp1002").mirrorY();

        copy1002left.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left.done().onTrue(m_robotCommands.shootShooterCommand());
        copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

        trenchLineUp1002.active().onTrue(m_robotCommands.stopShooterCommand());
        trenchLineUp1002.done().onTrue(copy1002left2.cmd());

        copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left2.done().onTrue(m_robotCommands.shootShooterCommand());

        routine.active().onTrue(
                Commands.sequence(
                        copy1002left.resetOdometry(),
                        copy1002left.cmd()));
        return routine;
    }

		public AutoRoutine bumpDepotMiddleLeft() {
        AutoRoutine routine = m_autoFactory.newRoutine("bumpDepotMiddleLeft");
        AutoTrajectory BumpToDepotLineup = routine.trajectory("BumpToDepotLineup");
        AutoTrajectory MiddleTurn = routine.trajectory("MiddleTurn");
        AutoTrajectory MiddleTurnBump = routine.trajectory("MiddleTurnBump");

        BumpToDepotLineup.atTime("Intake").onTrue(m_robotCommands.runIntake());
				BumpToDepotLineup.atTime("StopIntake").onTrue(m_robotCommands.stowIntake());
				BumpToDepotLineup.done().onTrue(m_robotCommands.shootShooterCommand());
				BumpToDepotLineup.doneDelayed(7).onTrue(MiddleTurn.cmd());

        MiddleTurn.active().onTrue(m_robotCommands.stowIntake());
        MiddleTurn.done().onTrue(MiddleTurnBump.cmd());

        routine.active().onTrue(
                Commands.sequence(
                        BumpToDepotLineup.resetOdometry(),
                        BumpToDepotLineup.cmd()));
        return routine;
    }

		public AutoRoutine trenchDepotMiddleLeft() {
        AutoRoutine routine = m_autoFactory.newRoutine("trenchDepotMiddleLeft");
        AutoTrajectory BumpToDepotLineup = routine.trajectory("BumpToDepotLineup");
        AutoTrajectory MiddleTurn = routine.trajectory("MiddleTurn");
        AutoTrajectory MiddleTurnTrench = routine.trajectory("MiddleTurnTrench");

        BumpToDepotLineup.atTime("Intake").onTrue(m_robotCommands.runIntake());
				BumpToDepotLineup.atTime("StopIntake").onTrue(m_robotCommands.stowIntake());
				BumpToDepotLineup.done().onTrue(m_robotCommands.shootShooterCommand());
				BumpToDepotLineup.doneDelayed(7).onTrue(MiddleTurn.cmd());

        MiddleTurn.active().onTrue(m_robotCommands.fill());
        MiddleTurn.done().onTrue(MiddleTurnTrench.cmd());

        routine.active().onTrue(
                Commands.sequence(
                        BumpToDepotLineup.resetOdometry(),
                        BumpToDepotLineup.cmd()));
        return routine;
    }

		public AutoRoutine centerHubMiddleLeft() {
        AutoRoutine routine = m_autoFactory.newRoutine("centerHubMiddleLeft");
        AutoTrajectory hubToDepotLineup = routine.trajectory("hubToDepotLineUp");
        AutoTrajectory MiddleTurn = routine.trajectory("MiddleTurn");
        AutoTrajectory MiddleTurnTrench = routine.trajectory("MiddleTurnTrench");

        hubToDepotLineup.atTime("Intake").onTrue(m_robotCommands.runIntake());
				hubToDepotLineup.atTime("StopIntake").onTrue(m_robotCommands.stowIntake());
				hubToDepotLineup.done().onTrue(m_robotCommands.shootShooterCommand());
				hubToDepotLineup.doneDelayed(7).onTrue(MiddleTurn.cmd());

        MiddleTurn.active().onTrue(m_robotCommands.fill());
        MiddleTurn.done().onTrue(MiddleTurnTrench.cmd());

        routine.active().onTrue(
                Commands.sequence(
                        hubToDepotLineup.resetOdometry(),
                        hubToDepotLineup.cmd()));
        return routine;
    }
}
