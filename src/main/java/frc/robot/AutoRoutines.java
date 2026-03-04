package frc.robot;

import java.util.function.Supplier;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final Supplier<Pose2d> m_poseSupplier;

  private final Field2d m_field = new Field2d();

  public AutoRoutines(AutoFactory autoFactory, Supplier<Pose2d> poseSupplier) {
    m_autoFactory = autoFactory;
    m_poseSupplier = poseSupplier;
    SmartDashboard.putData("Auto/Field", m_field);
  }

  /**
   * Draws the planned trajectory path on the Field2d widget once.
   * Uses Choreo.loadTrajectory() directly because AutoTrajectory does not
   * expose the underlying Pose2d array — collectPoses() does not exist.
   */
  private void setTrajectoryPoses(String name) {
    Choreo.<SwerveSample>loadTrajectory(name)
        .ifPresent(traj -> m_field.getObject("Trajectory").setPoses(traj.getPoses()));
  }

  /** Updates the live robot pose on the field widget each loop tick. */
  private void logRobotPose() {
    m_field.setRobotPose(m_poseSupplier.get());
  }

  public AutoRoutine depotToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToLeftStart");
    AutoTrajectory depotToLeftStart = routine.trajectory("DepotToLeftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("DepotToLeftStart")),
            depotToLeftStart.resetOdometry(),
            depotToLeftStart.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("DepotToMidLeft")),
            depotToMidLeft.resetOdometry(),
            depotToMidLeft.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine leftStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("LeftStartToCenter")),
            leftStartToCenter.resetOdometry(),
            leftStartToCenter.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine leftStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToDepot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("LeftStartToDepot")),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine midLeftToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidLeftToDepot");
    AutoTrajectory midLeftToDepot = routine.trajectory("MidLeftToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("MidLeftToDepot")),
            midLeftToDepot.resetOdometry(),
            midLeftToDepot.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine midRightToOutpost() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidRightToOutpost");
    AutoTrajectory midRightToOutpost = routine.trajectory("MidRightToOutpost");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("MidRightToOutpost")),
            midRightToOutpost.resetOdometry(),
            midRightToOutpost.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine midStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToDepot");
    AutoTrajectory midStartToDepot = routine.trajectory("MidStartToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("MidStartToDepot")),
            midStartToDepot.resetOdometry(),
            midStartToDepot.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine midStartToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToLeftStart");
    AutoTrajectory midStartToLeftStart = routine.trajectory("MidStartToLeftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("MidStartToLeftStart")),
            midStartToLeftStart.resetOdometry(),
            midStartToLeftStart.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine outpostToMidRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("OutpostToMidRight");
    AutoTrajectory outpostToMidRight = routine.trajectory("OutpostToMidRight");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("OutpostToMidRight")),
            outpostToMidRight.resetOdometry(),
            outpostToMidRight.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine rightStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToCenter");
    AutoTrajectory rightStartToCenter = routine.trajectory("RightStartToCenter");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("RightStartToCenter")),
            rightStartToCenter.resetOdometry(),
            rightStartToCenter.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine rightToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightToStealBalls");
    AutoTrajectory rightToStealBalls = routine.trajectory("RightToStealBalls");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("RightToStealBalls")),
            rightToStealBalls.resetOdometry(),
            rightToStealBalls.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine leftToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftToStealBalls");
    AutoTrajectory leftToStealBalls = routine.trajectory("LeftToStealBalls");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("LeftToStealBalls")),
            leftToStealBalls.resetOdometry(),
            leftToStealBalls.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine leftStartToRightStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToRightStart");
    AutoTrajectory leftStartToRightStart = routine.trajectory("LeftStartToRightStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("LeftStartToRightStart")),
            leftStartToRightStart.resetOdometry(),
            leftStartToRightStart.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }

  public AutoRoutine rightStartToleftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToleftStart");
    AutoTrajectory rightStartToleftStart = routine.trajectory("RightStartToleftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> setTrajectoryPoses("RightStartToleftStart")),
            rightStartToleftStart.resetOdometry(),
            rightStartToleftStart.cmd().deadlineWith(Commands.run(this::logRobotPose))));
    return routine;
  }
}