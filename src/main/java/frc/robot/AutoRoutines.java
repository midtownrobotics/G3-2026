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

  public AutoRoutines(AutoFactory autoFactory, RobotCommands robotCommands) {
    m_autoFactory = autoFactory;
    m_logger = new Logger(getClass());
    m_commands = robotCommands;
  }

  private void autoRoutineShoot(AutoTrajectory trajectory) {
    trajectory.active().onTrue(m_commands.revShooter());
    trajectory.doneDelayed(0.5).onTrue(m_commands.autoAimAndPrepareShootAutonomous());
  }

  public AutoRoutine depotToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToLeftStart");
    AutoTrajectory depotToLeftStart = routine.trajectory("DepotToLeftStart");
    
    autoRoutineShoot(depotToLeftStart);

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

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");

    autoRoutineShoot(depotToMidLeft);

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

    leftStartToCenter.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftStartToCenter.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    leftStartToCenter.atPose("shooting begins", 0.3, 0.4).onTrue(
        Commands.sequence(
          m_commands.revShooter(),
          m_commands.autoAimAndPrepareShootAutonomous()  
        )
    );

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

    leftStartToDepot.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    leftStartToDepot.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());
    
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

    midLeftToDepot.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
    midLeftToDepot.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());

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

      midStartToDepot.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
      midStartToDepot.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());


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
      
      rightStartToCenter.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
      rightStartToCenter.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());

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

      rightToStealBalls.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
      rightToStealBalls.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());


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

      leftToStealBalls.atPose("intaking begins", 0.3, 0.4).onTrue(m_commands.runIntake());
      leftToStealBalls.atPose("intaking ends", 0.3, 0.4).onTrue(m_commands.stowIntake());


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

  public AutoRoutine rightStartToLeftStart() {
      AutoRoutine routine = m_autoFactory.newRoutine("RightStartToLeftStart");
      AutoTrajectory rightStartToLeftStart = routine.trajectory("RightStartToLeftStart");

      routine.active().onTrue(
          Commands.sequence(
              Commands.runOnce(() -> {
                  rightStartToLeftStart.getInitialPose().ifPresent(pose -> m_logger.log("InitialPose", pose));
                  m_logger.log("ActiveTrajectory", "RightStartToLeftStart");
              }),
              rightStartToLeftStart.resetOdometry(),
              rightStartToLeftStart.cmd()));
      return routine;
  }
}