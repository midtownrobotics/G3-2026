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

    radialTrajectory.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    radialTrajectory.active().onTrue(m_robotCommands.shootShooterCommand().asProxy());
    radialTrajectory.done().onTrue(m_robotCommands.stopShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            radialTrajectory.resetOdometry(),
            radialTrajectory.cmd()));
    return routine;
  }

  public AutoRoutine tuneTangential() {
    AutoRoutine routine = m_autoFactory.newRoutine("TuneTangential");
    AutoTrajectory tangentialTrajectory = routine.trajectory("TangentialTrajectory");

    tangentialTrajectory.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    tangentialTrajectory.active().onTrue(m_robotCommands.shootShooterCommand().asProxy());
    tangentialTrajectory.done().onTrue(m_robotCommands.stopShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            tangentialTrajectory.resetOdometry(),
            tangentialTrajectory.cmd()));
    return routine;
  }

  public AutoRoutine pickupDepotAndShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");

    leftStartToDepot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    leftStartToDepot.active().onTrue(m_robotCommands.runIntake());
    leftStartToDepot.doneDelayed(1).onTrue(depotToShoot.cmd());
    depotToShoot.doneDelayed(0.5).onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine depotAndMiddleShoot() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToShoot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory depotToShoot = routine.trajectory("DepotToShoot");
    AutoTrajectory shootToCenter = routine.trajectory("ShootToCenter");

    leftStartToDepot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
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
            leftStartToDepot.resetOdometry(),
            leftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine middleAndDepotShootLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("Middle And Depot Shoot");
    AutoTrajectory startToCenterShoot = routine.trajectory("LeftStartToCenterToShoot");
    AutoTrajectory shootToDepotStraight = routine.trajectory("LeftShootToDepotStraightOn");

    startToCenterShoot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
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
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine middleAndDepotShootRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("Middle and Depot Shoot Right");
    AutoTrajectory startToCenterShoot = routine.trajectory("RightStartToCenterToShoot");

    startToCenterShoot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    startToCenterShoot.atTime(2.8).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.9).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.done().onTrue(m_robotCommands.shootShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine SOTMDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMDepot");
    AutoTrajectory SOTMLeftStartToDepot = routine.trajectory("SOTMLeftStartToDepot");

    SOTMLeftStartToDepot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    SOTMLeftStartToDepot.atTime(0.5).onTrue(m_robotCommands.shootShooterCommand().asProxy());
    SOTMLeftStartToDepot.atTime(0.67).onTrue(m_robotCommands.runIntake());
    SOTMLeftStartToDepot.atTime(4.0).onTrue(m_robotCommands.stowIntake());

    routine.active().onTrue(
        Commands.sequence(
            SOTMLeftStartToDepot.resetOdometry(),
            SOTMLeftStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine LeftDoubleSwipe() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftDoubleSwipe");
    AutoTrajectory LeftToCenterToShootAggressive = routine.trajectory("LeftToCenterToShootAggressive");
    AutoTrajectory LeftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory LeftStartCenterEnder = routine.trajectory("LeftStartCenterEnder");

    LeftToCenterToShootAggressive.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftToCenterToShootAggressive.atTime("startintake").onTrue(m_robotCommands.runIntake());
    LeftToCenterToShootAggressive.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftToCenterToShootAggressive.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    LeftToCenterToShootAggressive.doneDelayed(5).onTrue(LeftStartToDepot.cmd());

    LeftStartToDepot.active().onTrue(m_robotCommands.runIntake());
    LeftStartToDepot.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftStartToDepot.doneDelayed(3).onTrue(LeftStartCenterEnder.cmd());

    LeftStartCenterEnder.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            LeftToCenterToShootAggressive.resetOdometry(),
            LeftToCenterToShootAggressive.cmd()));
    return routine;
  }

  public AutoRoutine SOTMLeftCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMLeftCenter");
    AutoTrajectory startToCenterShoot = routine.trajectory("LeftStartToCenterToShoot");
    AutoTrajectory shootToDepotStraight = routine.trajectory("LeftShootToDepotStraightOn");

    startToCenterShoot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    startToCenterShoot.atTime(2.0).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.1).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.atTime(4.9).onTrue(m_robotCommands.shootShooterCommand().asProxy());
    startToCenterShoot.doneDelayed(3).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());

    routine.active().onTrue(
        Commands.sequence(
            startToCenterShoot.resetOdometry(),
            startToCenterShoot.cmd()));
    return routine;
  }

  public AutoRoutine SOTMLeftInverseTwice() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMLeftCenterInverse");
    AutoTrajectory startToCenterShootInverse = routine.trajectory("LeftStartToCenterToShootPassive");
    AutoTrajectory shootToDepotStraight = routine.trajectory("ShootStraightToDepotToShoot");
    AutoTrajectory startToCenterReturn = routine.trajectory("LeftToCenterReturn");

    startToCenterShootInverse.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    startToCenterShootInverse.atTime("startintake").onTrue(m_robotCommands.runIntake());
    startToCenterShootInverse.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    startToCenterShootInverse.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    startToCenterShootInverse.doneDelayed(3).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());
    shootToDepotStraight.doneDelayed(3).onTrue(startToCenterReturn.cmd());

    startToCenterReturn.active().onTrue(m_robotCommands.stopShooterCommand());
    startToCenterReturn.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            startToCenterShootInverse.resetOdometry(),
            startToCenterShootInverse.cmd()));
    return routine;
  }

  public AutoRoutine SOTMLeftTwice() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMLeftCenter");
    AutoTrajectory startToCenter = routine.trajectory("LeftToCenterToShootAggressive");
    AutoTrajectory shootToDepotStraight = routine.trajectory("ShootToDepotToShoot");
    AutoTrajectory startToCenterReturn = routine.trajectory("LeftToCenterReturn");

    startToCenter.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    startToCenter.atTime("startintake").onTrue(m_robotCommands.runIntake());
    startToCenter.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    startToCenter.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    startToCenter.doneDelayed(3).onTrue(shootToDepotStraight.cmd());

    shootToDepotStraight.active().onTrue(m_robotCommands.runIntake());
    shootToDepotStraight.done().onTrue(m_robotCommands.stowIntake());
    shootToDepotStraight.doneDelayed(0.5).onTrue(startToCenterReturn.cmd());

    startToCenterReturn.active().onTrue(m_robotCommands.stopShooterCommand());
    startToCenterReturn.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            startToCenter.resetOdometry(),
            startToCenter.cmd()));
    return routine;
  }

  public AutoRoutine SOTMRightTwice() {
    AutoRoutine routine = m_autoFactory.newRoutine("SOTMRightCenter");
    AutoTrajectory startToCenter = routine.trajectory("RightToCenterToShootAggressive");
    AutoTrajectory startToCenterReturn = routine.trajectory("RightToCenterReturn");

    startToCenter.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    startToCenter.atTime("startintake").onTrue(m_robotCommands.runIntake());
    startToCenter.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    startToCenter.atTime("startshooting").onTrue(m_robotCommands.shootShooterCommand());
    startToCenter.doneDelayed(8).onTrue(startToCenterReturn.cmd());

    startToCenterReturn.active().onTrue(m_robotCommands.stopShooterCommand());
    startToCenterReturn.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            startToCenter.resetOdometry(),
            startToCenter.cmd()));
    return routine;
  }
}
