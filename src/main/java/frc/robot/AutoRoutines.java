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

  public AutoRoutine pickupDepotAndShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");

    leftStartToDepot.active().onTrue(m_robotCommands.revShooter());
    leftStartToDepot.active().onTrue(m_robotCommands.runIntake());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.doneDelayed(0.5).onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement(),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine depotAndMiddleShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");
    AutoTrajectory shootToCenter = routine.trajectory("ShootToCenter");

    leftStartToDepot.active().onTrue(m_robotCommands.revShooter());
    leftStartToDepot.atTime(1).onTrue(m_robotCommands.runIntake());
    // leftStartToDepot.active().onTrue(m_robotCommands.zeroTurretHood());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());
    depotToShoot.doneDelayed(5).onTrue(m_robotCommands.idle());
    depotToShoot.doneDelayed(5).onTrue(shootToCenter.cmd());
    shootToCenter.atTime(2.5).onTrue(m_robotCommands.runIntake());
    shootToCenter.atTime(6.9).onTrue(m_robotCommands.stowIntake());
    shootToCenter.done().onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement(),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine middleAndDepotShootLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("Middle And Depot Shoot");
    AutoTrajectory startToCenterShoot = routine.trajectory("LeftStartToCenterToShoot");
    AutoTrajectory shootToDepotStraight = routine.trajectory("LeftShootToDepotStraightOn");
    
    startToCenterShoot.atTime(2.0).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.1).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.done().onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());
    startToCenterShoot.doneDelayed(5).onFalse(m_robotCommands.idle());
    startToCenterShoot.doneDelayed(5).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
      Commands.sequence(
        m_robotCommands.stowIntakeAndHaltTurretMovement(),
        startToCenterShoot.resetOdometry(),
        startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine middleAndDepotShootRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("Middle and Depot Shoot Right");
    AutoTrajectory startToCenterShoot = routine.trajectory("RightStartToCenterToShoot");

    startToCenterShoot.atTime(2.8).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.9).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.done().onTrue(m_robotCommands.autoAimAndPrepareShootAutonomous());

    routine.active().onTrue(
      Commands.sequence(
        m_robotCommands.stowIntakeAndHaltTurretMovement(),
        startToCenterShoot.resetOdometry(), 
        startToCenterShoot.cmd()));
    return routine;
  }
}
