package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.GeometryUtil;
import frc.lib.LoggedTunableNumber;
import frc.robot.commands.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {

    private final AutoFactory m_autoFactory;
    private final RobotCommands m_robotCommands;
    private final LoggedTunableNumber m_hubSwipeDelaySeconds = new LoggedTunableNumber("HubSwipeDelaySeconds", 0.0);
    private final FollowPath.Builder pathBuilder;
    private final Drive m_drive;

    private static final double kTrenchHeadingRad = 0;
    private static final Rotation2d kTrenchHeading = Rotation2d.fromRadians(kTrenchHeadingRad);
    private static final Rotation2d kTrenchHeadingMirrored = Rotation2d.fromRadians(-kTrenchHeadingRad);
    private static final Pose2d kTrenchEntryRight = new Pose2d(4.334, 0.585, kTrenchHeading);
    private static final Pose2d kTrenchExitRight = new Pose2d(6.589, 0.795, kTrenchHeading);

    public AutoRoutines(AutoFactory autoFactory, RobotCommands robotCommands,
            Drive drive) {
        m_autoFactory = autoFactory;
        m_robotCommands = robotCommands;
        m_drive = drive;

        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                4.729,   
                12.044,  
                682.5,   
                2945.6,  
                Inches.of(1).in(Meters),    
                2.0,
                0.3));   

        pathBuilder = new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(7.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(4.0, 0.0, 0.0));
    }

    public Command driveToPose(Pose2d target) {
        return pathBuilder.build(new Path(new Path.Waypoint(target)));
    }

    public Command trenchSupport() {
        return Commands.defer(() -> {
            Pose2d current = m_drive.getPose();

            Pose2d entry = GeometryUtil.flip(kTrenchEntryRight);
            Pose2d exit = GeometryUtil.flip(kTrenchExitRight);

            Path path = new Path(
                    new PathConstraints()
                            .setMaxVelocityMetersPerSec(0.5)
                            .setMaxAccelerationMetersPerSec2(1.0),
                    new Path.Waypoint(current),
                    new Path.Waypoint(entry, 0.4),
                    new Path.Waypoint(exit));

            return pathBuilder.build(path);
        }, Set.of(m_drive));
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
				BackwardsBump.doneDelayed(0.3).onTrue(m_robotCommands.startShootingCommand());
        BackwardsBump.doneDelayed(0.5).onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.atTime("PrepareForSweep").onTrue(m_robotCommands.fill());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

				BackwardsBump2.done().onTrue(m_robotCommands.revShooterCommand());
        BackwardsBump2.doneDelayed(0.3).onTrue(m_robotCommands.startShootingCommand());

        routine.active().onTrue(
                Commands.sequence(
												m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(0.5)),
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
				BackwardsBump.doneDelayed(0.3).onTrue(m_robotCommands.startShootingCommand());
        BackwardsBump.doneDelayed(0.5).onTrue(BumpToTrenchSOTM.cmd());

        BumpToTrenchSOTM.atTime("PrepareForSweep").onTrue(m_robotCommands.fill());
        BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

        TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

				BackwardsBump2.done().onTrue(m_robotCommands.revShooterCommand());
        BackwardsBump2.doneDelayed(0.3).onTrue(m_robotCommands.startShootingCommand());

        routine.active().onTrue(
                Commands.sequence(
												m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(0.5)),
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
                        Commands.defer(() -> Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
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
                        Commands.defer(() -> Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
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
        //AutoTrajectory DepotMiddle = routine.trajectory("DepotMiddle");

        CenterDepot.active().onTrue(m_robotCommands.runIntake());
        CenterDepot.active().onTrue(m_robotCommands.revShooterCommand());
				CenterDepot.done().onTrue(m_robotCommands.startShootingCommand());
        CenterDepot.doneDelayed(8);

      //  DepotMiddle.active().onTrue(m_robotCommands.fill());

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
