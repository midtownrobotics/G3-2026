package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Logger;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final Robot robot;
  private final Logger m_logger;

  public AutoRoutines(AutoFactory autoFactory, Robot robot) {
    m_autoFactory = autoFactory;
    this.robot = robot;
    m_logger = new Logger(getClass());
  }

  public AutoRoutine depotToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToLeftStart");
    AutoTrajectory depotToLeftStart = routine.trajectory("DepotToLeftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              depotToLeftStart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "DepotToLeftStart");
            }),
            depotToLeftStart.resetOdometry(),
            depotToLeftStart.cmd()));
    return routine;
  }

  public AutoRoutine pickupDepotAndShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");

    leftStartToDepot.active().onTrue(robot.revFlywheels());
    leftStartToDepot.active().onTrue(robot.runIntakeCommand());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.doneDelayed(0.5).onTrue(robot.shoot());

    routine.active().onTrue(
        Commands.sequence(
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              depotToMidLeft.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "DepotToMidLeft");
            }),
            depotToMidLeft.resetOdometry(),
            depotToMidLeft.cmd()));
    return routine;
  }

  public AutoRoutine leftStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              leftStartToCenter.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftStartToCenter");
            }),
            leftStartToCenter.resetOdometry(),
            leftStartToCenter.cmd()));
    return routine;
  }

  public AutoRoutine leftStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToDepot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              leftStartToDepot.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftStartToDepot");
            }),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine midLeftToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidLeftToDepot");
    AutoTrajectory midLeftToDepot = routine.trajectory("MidLeftToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midLeftToDepot.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "MidLeftToDepot");
            }),
            midLeftToDepot.resetOdometry(),
            midLeftToDepot.cmd()));
    return routine;
  }

  public AutoRoutine midRightToOutpost() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidRightToOutpost");
    AutoTrajectory midRightToOutpost = routine.trajectory("MidRightToOutpost");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midRightToOutpost.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "MidRightToOutpost");
            }),
            midRightToOutpost.resetOdometry(),
            midRightToOutpost.cmd()));
    return routine;
  }

  public AutoRoutine midStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToDepot");
    AutoTrajectory midStartToDepot = routine.trajectory("MidStartToDepot");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midStartToDepot.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "MidStartToDepot");
            }),
            midStartToDepot.resetOdometry(),
            midStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine midStartToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToLeftStart");
    AutoTrajectory midStartToLeftStart = routine.trajectory("MidStartToLeftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midStartToLeftStart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "MidStartToLeftStart");
            }),
            midStartToLeftStart.resetOdometry(),
            midStartToLeftStart.cmd()));
    return routine;
  }

  public AutoRoutine outpostToMidRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("OutpostToMidRight");
    AutoTrajectory outpostToMidRight = routine.trajectory("OutpostToMidRight");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              outpostToMidRight.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "OutpostToMidRight");
            }),
            outpostToMidRight.resetOdometry(),
            outpostToMidRight.cmd()));
    return routine;
  }

  public AutoRoutine rightStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToCenter");
    AutoTrajectory rightStartToCenter = routine.trajectory("RightStartToCenter");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              rightStartToCenter.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "RightStartToCenter");
            }),
            rightStartToCenter.resetOdometry(),
            rightStartToCenter.cmd()));
    return routine;
  }

  public AutoRoutine rightToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightToStealBalls");
    AutoTrajectory rightToStealBalls = routine.trajectory("RightToStealBalls");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              rightToStealBalls.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "RightToStealBalls");
            }),
            rightToStealBalls.resetOdometry(),
            rightToStealBalls.cmd()));
    return routine;
  }

  public AutoRoutine leftToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftToStealBalls");
    AutoTrajectory leftToStealBalls = routine.trajectory("LeftToStealBalls");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              leftToStealBalls.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftToStealBalls");
            }),
            leftToStealBalls.resetOdometry(),
            leftToStealBalls.cmd()));
    return routine;
  }

  public AutoRoutine leftStartToRightStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToRightStart");
    AutoTrajectory leftStartToRightStart = routine.trajectory("LeftStartToRightStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              leftStartToRightStart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftStartToRightStart");
            }),
            leftStartToRightStart.resetOdometry(),
            leftStartToRightStart.cmd()));
    return routine;
  }

  public AutoRoutine rightStartToleftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToleftStart");
    AutoTrajectory rightStartToleftStart = routine.trajectory("RightStartToleftStart");

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              rightStartToleftStart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "RightStartToleftStart");
            }),
            rightStartToleftStart.resetOdometry(),
            rightStartToleftStart.cmd()));
    return routine;
  }
}