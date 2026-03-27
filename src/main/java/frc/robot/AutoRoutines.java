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

  public AutoRoutine tuneRadial() {
    AutoRoutine routine = m_autoFactory.newRoutine("TuneRadial");
    AutoTrajectory radialTrajectory = routine.trajectory("RadialTrajectory");

    radialTrajectory.active().onTrue(m_robotCommands.shooterTrackShootingParamters().asProxy());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            radialTrajectory.resetOdometry(),
            radialTrajectory.cmd()));
    return routine;
  }

  public AutoRoutine tuneTangential() {
    AutoRoutine routine = m_autoFactory.newRoutine("TuneTangential");
    AutoTrajectory tangentialTrajectory = routine.trajectory("TangentialTrajectory");

    tangentialTrajectory.active().onTrue(m_robotCommands.shooterTrackShootingParamters().asProxy());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            tangentialTrajectory.resetOdometry(),
            tangentialTrajectory.cmd()));
    return routine;
  }

  public AutoRoutine pickupDepotAndShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");

    leftStartToDepot.active().onTrue(m_robotCommands.runIntake());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.doneDelayed(0.5).onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine depotAndMiddleShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");
    AutoTrajectory shootToCenter = routine.trajectory("ShootToCenter");

    leftStartToDepot.atTime(1).onTrue(m_robotCommands.runIntake());
    // leftStartToDepot.active().onTrue(m_robotCommands.zeroTurretHood());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(m_robotCommands.shootShooterCommand());
    depotToShoot.doneDelayed(5).onTrue(m_robotCommands.idle());
    depotToShoot.doneDelayed(5).onTrue(shootToCenter.cmd());
    shootToCenter.atTime(2.5).onTrue(m_robotCommands.runIntake());
    shootToCenter.atTime(6.9).onTrue(m_robotCommands.stowIntake());
    shootToCenter.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
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
    startToCenterShoot.done().onTrue(m_robotCommands.shootShooterCommand());
    startToCenterShoot.doneDelayed(5).onFalse(m_robotCommands.idle());
    startToCenterShoot.doneDelayed(5).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine middleAndDepotShootRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("Middle and Depot Shoot Right");
    AutoTrajectory startToCenterShoot = routine.trajectory("RightStartToCenterToShoot");

    startToCenterShoot.atTime(2.8).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.9).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine SOTMDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMDepot");
    AutoTrajectory SOTMLeftStartToDepot = routine.trajectory("SOTMLeftStartToDepot");

    SOTMLeftStartToDepot.atTime(0.5).onTrue(m_robotCommands.shootShooterCommand().asProxy());
    SOTMLeftStartToDepot.atTime(0.67).onTrue(m_robotCommands.runIntake());
    SOTMLeftStartToDepot.atTime(4.0).onTrue(m_robotCommands.stowIntake());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            SOTMLeftStartToDepot.resetOdometry(),
            SOTMLeftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine SOTMLeftCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMLeftCenter");
    AutoTrajectory startToCenterShoot = routine.trajectory("LeftStartToCenterToShoot");
    AutoTrajectory shootToDepotStraight = routine.trajectory("LeftShootToDepotStraightOn");

    startToCenterShoot.atTime(2.0).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.1).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.atTime(6.2).onTrue(m_robotCommands.shootShooterCommand().asProxy());
    startToCenterShoot.doneDelayed(3).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine SOTMLeftTwice() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMLeftCenter");
    AutoTrajectory startToCenter = routine.trajectory("AlbanyCenterLeft");
    AutoTrajectory startToCenterReturn = routine.trajectory("AlbanyReturnLeft");

    startToCenter.atTime("startintake").onTrue(m_robotCommands.runIntake());
    startToCenter.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    startToCenter.atTime("startshooting").onTrue(m_robotCommands.shootShooterCommand());
    startToCenter.doneDelayed(8).onTrue(startToCenterReturn.cmd());

    startToCenterReturn.active().onTrue(m_robotCommands.stopShooterCommand());
    startToCenterReturn.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            startToCenter.resetOdometry(),
            startToCenter.cmd()));
    return routine;
  }

  public AutoRoutine SOTMRightTwice() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMRightCenter");
    AutoTrajectory startToCenter = routine.trajectory("AlbanyCenterRight");
    AutoTrajectory startToCenterReturn = routine.trajectory("AlbanyReturnRight");

    startToCenter.atTime("startintake").onTrue(m_robotCommands.runIntake());
    startToCenter.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    startToCenter.atTime("startshooting").onTrue(m_robotCommands.shootShooterCommand());
    startToCenter.doneDelayed(8).onTrue(startToCenterReturn.cmd());

    startToCenterReturn.active().onTrue(m_robotCommands.stopShooterCommand());
    startToCenterReturn.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy(),
            startToCenter.resetOdometry(),
            startToCenter.cmd()));
    return routine;
  }
}
