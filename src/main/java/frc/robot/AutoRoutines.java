package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Logger;
import frc.robot.commands.RobotCommands;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final Logger m_logger;
  private final RobotCommands m_commands;

  public AutoRoutines(AutoFactory autoFactory, Robot robot, RobotCommands robotCommands) {
    m_autoFactory = autoFactory;
    m_logger = new Logger(getClass());
    m_commands = robotCommands;
  }

  private void autoRoutineShoot(AutoTrajectory trajectory) {
    trajectory.active().onTrue(m_commands.revShooter());
    trajectory.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());
  }
  
  public AutoRoutine leftStartDepotPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToDepotPath");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    leftStartToDepot.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftStartToDepot.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());

    depotToMidLeft.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());
    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              leftStartToDepot.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "leftStartToDepot");
            }),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd(),
            Commands.runOnce(() -> {
              depotToMidLeft.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "depotToMidLeft");
            }),
            depotToMidLeft.cmd()));

    return routine;
  }

  public AutoRoutine leftStartCenterPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");

    leftStartToCenter.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftStartToCenter.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    leftStartToCenter.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

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

  public AutoRoutine leftStartStealBallsPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftToStealBalls");
    AutoTrajectory leftToStealBalls = routine.trajectory("LeftToStealBalls");

    leftToStealBalls.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftToStealBalls.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    leftToStealBalls.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

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

  public AutoRoutine midStartCenterPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartCenterPath");
    AutoTrajectory midStartToLeftstart = routine.trajectory("MidStartToLeftStart");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");

    leftStartToCenter.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftStartToCenter.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    leftStartToCenter.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midStartToLeftstart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "midStartToLeftstart");
            }),
            midStartToLeftstart.resetOdometry(),
            midStartToLeftstart.cmd(),
            Commands.runOnce(() -> {
              leftStartToCenter.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "leftStartToCenter");
            }),
            leftStartToCenter.cmd()));

    return routine;
  }

  public AutoRoutine midStartDepotPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartDepotPath");
    AutoTrajectory midStartToDepot = routine.trajectory("MidStartToDepot");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    midStartToDepot.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    midStartToDepot.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    depotToMidLeft.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              midStartToDepot.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "midStartToDepot");
            }),
            midStartToDepot.resetOdometry(),
            midStartToDepot.cmd(),
            Commands.runOnce(() -> {
              depotToMidLeft.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "depotToMidLeft");
            }),
            depotToMidLeft.cmd()));

    return routine;
  }

  public AutoRoutine rightStartCenterPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartCenterPath");
    AutoTrajectory rightStartToCenter = routine.trajectory("RightStartToCenter");

    rightStartToCenter.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    rightStartToCenter.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    rightStartToCenter.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              rightStartToCenter.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftStartToCenter");
            }),
            rightStartToCenter.resetOdometry(),
            rightStartToCenter.cmd()));
    return routine;
  }

  public AutoRoutine rightStartStealBallsPath() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightToStealBalls");
    AutoTrajectory rightToStealBalls = routine.trajectory("RightToStealBalls");

    rightToStealBalls.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    rightToStealBalls.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    rightToStealBalls.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
              rightToStealBalls.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
              m_logger.log("ActiveTrajectory", "LeftToStealBalls");
            }),
            rightToStealBalls.resetOdometry(),
            rightToStealBalls.cmd()));
    return routine;
  }


  
}