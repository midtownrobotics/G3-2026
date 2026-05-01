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

    public AutoRoutines(AutoFactory autoFactory, RobotCommands robotCommands) {
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
        TrenchSweep.done().onTrue(BackwardsBump.cmd());

				BackwardsBump.done().onTrue(m_robotCommands.revShooterCommand());
				BackwardsBump.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());
        BackwardsBump.doneDelayed(1).onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.atTime(7.5).onTrue(m_robotCommands.fill());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

				BackwardsBump2.doneDelayed(0.5).onTrue(m_robotCommands.revShooterCommand());
        BackwardsBump2.doneDelayed(1).onTrue(m_robotCommands.startShootingCommand());

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
        TrenchSweep.done().onTrue(BackwardsBump.cmd());

				BackwardsBump.done().onTrue(m_robotCommands.revShooterCommand());
				BackwardsBump.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());
        BackwardsBump.doneDelayed(1).onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.atTime(7.5).onTrue(m_robotCommands.fill());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

				BackwardsBump2.doneDelayed(0.5).onTrue(m_robotCommands.revShooterCommand());
        BackwardsBump2.doneDelayed(1).onTrue(m_robotCommands.startShootingCommand());

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
        HubSwipe.done().onTrue(m_robotCommands.revShooterCommand());
        HubSwipe.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());

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
        HubSwipe.done().onTrue(m_robotCommands.revShooterCommand());
        HubSwipe.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());

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
        copy1002left.done().onTrue(m_robotCommands.revShooterCommand());
        copy1002left.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());
        copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

        trenchLineUp1002.active().onTrue(m_robotCommands.stopShooterCommand());
        trenchLineUp1002.done().onTrue(copy1002left2.cmd());

        copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left2.done().onTrue(m_robotCommands.revShooterCommand());
        copy1002left2.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());

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
        copy1002left.done().onTrue(m_robotCommands.revShooterCommand());
        copy1002left.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());
        copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

        trenchLineUp1002.active().onTrue(m_robotCommands.stopShooterCommand());
        trenchLineUp1002.done().onTrue(copy1002left2.cmd());

        copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
        copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
        copy1002left2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
        copy1002left2.done().onTrue(m_robotCommands.revShooterCommand());
        copy1002left2.doneDelayed(0.5).onTrue(m_robotCommands.startShootingCommand());

        routine.active().onTrue(
                Commands.sequence(
                        copy1002left.resetOdometry(),
                        copy1002left.cmd()));
        return routine;
    }

		public AutoRoutine match13Depot() {
        AutoRoutine routine = m_autoFactory.newRoutine("match13Depot");
        AutoTrajectory CenterDepot = routine.trajectory("CenterDepot");
        AutoTrajectory DepotMiddle = routine.trajectory("DepotMiddle");

        CenterDepot.active().onTrue(m_robotCommands.runIntake());
        CenterDepot.active().onTrue(m_robotCommands.revShooterCommand());
				CenterDepot.done().onTrue(m_robotCommands.startShootingCommand());
        CenterDepot.doneDelayed(8).onTrue(DepotMiddle.cmd());

        DepotMiddle.active().onTrue(m_robotCommands.fill());

        routine.active().onTrue(
                Commands.sequence(
                        CenterDepot.resetOdometry(),
                        CenterDepot.cmd()));
        return routine;
    }

		public AutoRoutine rightHubCleanUp() {
        AutoRoutine rightHubCleanUp = m_autoFactory.newRoutine("rightHubCleanUp");
        AutoTrajectory RightTrenchToCenterBack = rightHubCleanUp.trajectory("RightTrenchToCenterBack");
        AutoTrajectory RightHubCleanup = rightHubCleanUp.trajectory("RightHubCleanup");
				AutoTrajectory BackwardsBump = rightHubCleanUp.trajectory("BackwardsBump").mirrorY(); 
				AutoTrajectory LeftBumpToDepot = rightHubCleanUp.trajectory("LeftBumpToDepot");

				RightTrenchToCenterBack.done().onTrue(RightHubCleanup.cmd());
				RightHubCleanup.done().onTrue(BackwardsBump.cmd());
				BackwardsBump.atTime(0.7).onTrue(m_robotCommands.revShooterCommand());
				BackwardsBump.done().onTrue(LeftBumpToDepot.cmd());
				LeftBumpToDepot.active().onTrue(m_robotCommands.startShootingCommand());
				LeftBumpToDepot.atTime("stopShooting").onTrue(m_robotCommands.fill());
				LeftBumpToDepot.atTime("startShooting").onTrue(m_robotCommands.startShootingCommand());

        rightHubCleanUp.active().onTrue(
                Commands.sequence(
												m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(2)),
                        RightTrenchToCenterBack.resetOdometry(),
                        RightTrenchToCenterBack.cmd()));
        return rightHubCleanUp;
    }
}
