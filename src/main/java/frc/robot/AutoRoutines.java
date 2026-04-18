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

    startToCenterShoot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement()
        .andThen(m_robotCommands.haltTurretAndHoodMovement()).asProxy());
    startToCenterShoot.atTime(2.0).onTrue(m_robotCommands.runIntake());
    startToCenterShoot.atTime(4.1).onTrue(m_robotCommands.stowIntake());
    startToCenterShoot.doneDelayed(1).onTrue(m_robotCommands.shootShooterCommand());
    startToCenterShoot.doneDelayed(4).onFalse(m_robotCommands.idle());
    startToCenterShoot.doneDelayed(4).onTrue(shootToDepotStraight.cmd());

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

  public AutoRoutine LeftDoubleSwipeDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftDoubleSwipe");
    AutoTrajectory LeftStartToBump = routine.trajectory("LeftStartToBump");
    AutoTrajectory CenterToShootAggressive = routine.trajectory("CenterToShootAggressive");
    AutoTrajectory LeftStartToDepot = routine.trajectory("LeftStartToDepot");
    AutoTrajectory LeftStartCenterEnder = routine.trajectory("LeftStartCenterEnder");

    LeftStartToBump.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftStartToBump.done().onTrue(CenterToShootAggressive.cmd());

    CenterToShootAggressive.active().onTrue(m_robotCommands.runIntake());
    CenterToShootAggressive.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    CenterToShootAggressive.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    CenterToShootAggressive.doneDelayed(3).onTrue(LeftStartToDepot.cmd());

    LeftStartToDepot.active().onTrue(m_robotCommands.runIntake());
    LeftStartToDepot.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftStartToDepot.doneDelayed(1).onTrue(LeftStartCenterEnder.cmd());

    LeftStartCenterEnder.atTime("startintake").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            LeftStartToBump.resetOdometry(),
            LeftStartToBump.cmd()));
    return routine;
  }

  public AutoRoutine CenterStartDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("CenterStartDepotOutpost");
    AutoTrajectory CenterStartToDepot = routine.trajectory("CenterStartToDepot");
    AutoTrajectory LeftStartToBump = routine.trajectory("LeftStartToBump");

    CenterStartToDepot.active().onTrue(m_robotCommands.shootShooterCommand().asProxy());
    CenterStartToDepot.atTime("startintake").onTrue(m_robotCommands.runIntake());
    CenterStartToDepot.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    CenterStartToDepot.doneDelayed(8).onTrue(LeftStartToBump.cmd());

    LeftStartToBump.active().onTrue(m_robotCommands.stopShooterCommand());

    routine.active().onTrue(
        Commands.sequence(
            CenterStartToDepot.resetOdometry(),
            CenterStartToDepot.cmd()));
    return routine;
  }

  public AutoRoutine RightDoubleSwipe() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightDoubleSwipe");
    AutoTrajectory LeftToCenterToShootAggressive = routine.trajectory("LeftToCenterToShootAggressive").mirrorY();
    AutoTrajectory LeftToCenterToShootAggressive2 = routine.trajectory("LeftToCenterToShootAggressive").mirrorY();

    LeftToCenterToShootAggressive.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftToCenterToShootAggressive.atTime("startintake").onTrue(m_robotCommands.runIntake());
    LeftToCenterToShootAggressive.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftToCenterToShootAggressive.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    LeftToCenterToShootAggressive.doneDelayed(3).onTrue(LeftToCenterToShootAggressive2.cmd());

    LeftToCenterToShootAggressive2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    LeftToCenterToShootAggressive2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftToCenterToShootAggressive2.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());

    routine.active().onTrue(
        Commands.sequence(
            LeftToCenterToShootAggressive.resetOdometry(),
            LeftToCenterToShootAggressive.cmd()));
    return routine;
  }

  public AutoRoutine LeftStartFeeding() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartFeeding");
    AutoTrajectory LeftStartFeeding = routine.trajectory("LeftStartFeeding");

    LeftStartFeeding.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftStartFeeding.atTime("starteverything").onTrue(m_robotCommands.runIntake());
    LeftStartFeeding.atTime("starteverything").onTrue(m_robotCommands.shootShooterCommand().asProxy());

    routine.active().onTrue(
        Commands.sequence(
            LeftStartFeeding.resetOdometry(),
            LeftStartFeeding.cmd()));
    return routine;
  }

  public AutoRoutine LeftStartFeedingDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartFeedingDepot");
    AutoTrajectory LeftStartFeedingDepot = routine.trajectory("LeftStartFeedingDepot");
    AutoTrajectory FeedingDepot = routine.trajectory("FeedingDepot");

    FeedingDepot.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    FeedingDepot.atTime("startshooting").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    FeedingDepot.atTime("startintake").onTrue(m_robotCommands.runIntake());
    FeedingDepot.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    FeedingDepot.doneDelayed(1.5).onTrue(LeftStartFeedingDepot.cmd());

    LeftStartFeedingDepot.atTime("starteverything").onTrue(m_robotCommands.runIntake());

    routine.active().onTrue(
        Commands.sequence(
            FeedingDepot.resetOdometry(),
            FeedingDepot.cmd()));
    return routine;
  }

  public AutoRoutine RightStartFeeding() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartFeeding");
    AutoTrajectory LeftStartFeeding = routine.trajectory("LeftStartFeeding").mirrorY();

    LeftStartFeeding.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftStartFeeding.atTime("starteverything").onTrue(m_robotCommands.runIntake());
    LeftStartFeeding.atTime("starteverything").onTrue(m_robotCommands.shootShooterCommand().asProxy());

    routine.active().onTrue(
        Commands.sequence(
            LeftStartFeeding.resetOdometry(),
            LeftStartFeeding.cmd()));
    return routine;
  }

  public AutoRoutine LeftDoubleSwipe() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftDoubleSwipe");
    AutoTrajectory LeftToCenterToShootAggressive = routine.trajectory("LeftToCenterToShootAggressive");
    AutoTrajectory LeftToCenterToShootAggressive2 = routine.trajectory("LeftToCenterToShootAggressive");

    LeftToCenterToShootAggressive.active().onTrue(m_robotCommands.stowIntakeAndHaltTurretMovement().asProxy());
    LeftToCenterToShootAggressive.atTime("startintake").onTrue(m_robotCommands.runIntake());
    LeftToCenterToShootAggressive.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftToCenterToShootAggressive.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());
    LeftToCenterToShootAggressive.doneDelayed(3).onTrue(LeftToCenterToShootAggressive2.cmd());

    LeftToCenterToShootAggressive2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    LeftToCenterToShootAggressive2.atTime("stopintake").onTrue(m_robotCommands.stowIntake());
    LeftToCenterToShootAggressive2.atTime("startshoot").onTrue(m_robotCommands.shootShooterCommand().asProxy());

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

  public AutoRoutine testPath(int id, boolean slow) {
    AutoRoutine routine = m_autoFactory.newRoutine("TestPath" + id);
    AutoTrajectory trajectory = routine.trajectory("Test" + id + (slow ? "_Slow" : ""));

    routine.active().onTrue(
        Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()));
    return routine;
  }
}
