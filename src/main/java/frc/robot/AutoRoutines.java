package frc.robot;

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
import frc.robot.commands.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final RobotCommands m_robotCommands;
  private final FollowPath.Builder pathBuilder;
  private final CommandSwerveDrivetrain m_drive;

  private static final double kTrenchHeadingRad = 0;
  private static final Rotation2d kTrenchHeading = Rotation2d.fromRadians(kTrenchHeadingRad);
  private static final Rotation2d kTrenchHeadingMirrored = Rotation2d.fromRadians(-kTrenchHeadingRad);

  private static final Pose2d kTrenchEntryRight = new Pose2d(4.334, 0.585, kTrenchHeading);
  private static final Pose2d kTrenchExitRight = new Pose2d(6.589, 0.795, kTrenchHeading);

  public AutoRoutines(AutoFactory autoFactory, Robot robot, RobotCommands robotCommands,
      CommandSwerveDrivetrain drive) {
    m_autoFactory = autoFactory;
    m_robotCommands = robotCommands;
    m_drive = drive;

    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        4.729,   // maxVelocityMetersPerSec
        12.044,  // maxAccelerationMetersPerSec2
        682.5,   // maxVelocityDegPerSec
        2945.6,  // maxAccelerationDegPerSec2
        0.05,    // endTranslationToleranceMeters (5 cm)
        2.0,     // endRotationToleranceDeg
        0.3));   // intermediateHandoffRadiusMeters

    pathBuilder = new FollowPath.Builder(
        drive,
        drive::getPose,
        drive::getChassisSpeeds,
        drive::drive,
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(3.0, 0.0, 0.0),
        new PIDController(2.0, 0.0, 0.0));
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
