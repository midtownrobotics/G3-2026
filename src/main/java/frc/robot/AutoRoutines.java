package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Logger;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Logger m_log = new Logger(getClass());

  public AutoRoutines(AutoFactory autoFactory, Supplier<Pose2d> poseSupplier) {
    m_autoFactory = autoFactory;
    m_poseSupplier = poseSupplier;
  }

  public AutoRoutine depotToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToLeftStart");
    AutoTrajectory depotToLeftStart = routine.trajectory("DepotToLeftStart");

    routine.active().onTrue(
      Commands.sequence(
        depotToLeftStart.resetOdometry(),
        depotToLeftStart.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    routine.active().onTrue(
      Commands.sequence(
        depotToMidLeft.resetOdometry(),
        depotToMidLeft.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine leftStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");

    routine.active().onTrue(
      Commands.sequence(
        leftStartToCenter.resetOdometry(),
        leftStartToCenter.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine leftStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToDepot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");

    routine.active().onTrue(
      Commands.sequence(
        leftStartToDepot.resetOdometry(),
        leftStartToDepot.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine midLeftToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidLeftToDepot");
    AutoTrajectory midLeftToDepot = routine.trajectory("MidLeftToDepot");

    routine.active().onTrue(
      Commands.sequence(
        midLeftToDepot.resetOdometry(),
        midLeftToDepot.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine midRightToOutpost() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidRightToOutpost");
    AutoTrajectory midRightToOutpost = routine.trajectory("MidRightToOutpost");

    routine.active().onTrue(
      Commands.sequence(
        midRightToOutpost.resetOdometry(),
        midRightToOutpost.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine midStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToDepot");
    AutoTrajectory midStartToDepot = routine.trajectory("MidStartToDepot");

    routine.active().onTrue(
      Commands.sequence(
        midStartToDepot.resetOdometry(),
        midStartToDepot.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine midStartToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToLeftStart");
    AutoTrajectory midStartToLeftStart = routine.trajectory("MidStartToLeftStart");

    routine.active().onTrue(
      Commands.sequence(
        midStartToLeftStart.resetOdometry(),
        midStartToLeftStart.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine outpostToMidRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("OutpostToMidRight");
    AutoTrajectory outpostToMidRight = routine.trajectory("OutpostToMidRight");

    routine.active().onTrue(
      Commands.sequence(
        outpostToMidRight.resetOdometry(),
        outpostToMidRight.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine rightStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToCenter");
    AutoTrajectory rightStartToCenter = routine.trajectory("RightStartToCenter");

    routine.active().onTrue(
      Commands.sequence(
        rightStartToCenter.resetOdometry(),
        rightStartToCenter.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine rightToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightToStealBalls");
    AutoTrajectory rightToStealBalls = routine.trajectory("RightToStealBalls");

    routine.active().onTrue(
      Commands.sequence(
        rightToStealBalls.resetOdometry(),
        rightToStealBalls.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine leftToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftToStealBalls");
    AutoTrajectory leftToStealBalls = routine.trajectory("LeftToStealBalls");

    routine.active().onTrue(
      Commands.sequence(
        leftToStealBalls.resetOdometry(),
        leftToStealBalls.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine leftStartToRightStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToRightStart");
    AutoTrajectory leftStartToRightStart = routine.trajectory("LeftStartToRightStart");

    routine.active().onTrue(
      Commands.sequence(
        leftStartToRightStart.resetOdometry(),
        leftStartToRightStart.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }

  public AutoRoutine rightStartToleftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToleftStart");
    AutoTrajectory rightStartToleftStart = routine.trajectory("RightStartToleftStart");

    routine.active().onTrue(
      Commands.sequence(
        rightStartToleftStart.resetOdometry(),
        rightStartToleftStart.cmd().deadlineWith(
          Commands.run(() -> m_log.log("Auto/CurrentPath", m_poseSupplier.get())))));
    return routine;
  }
}