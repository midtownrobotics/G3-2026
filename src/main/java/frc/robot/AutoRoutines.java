package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;

  public AutoRoutines(AutoFactory autoFactory) {
    m_autoFactory = autoFactory;
  }

  public AutoRoutine depotToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToLeftStart");
    AutoTrajectory depotToLeftStart = routine.trajectory("DepotToLeftStart");
    
    DogLog.log("Auto/CurrentPath", depotToLeftStart.active().getAsBoolean());

    routine.active().onTrue(
      Commands.sequence(
        depotToLeftStart.resetOdometry(),
        depotToLeftStart.cmd()));
    return routine;
  }

  public AutoRoutine depotToMidLeft() {
    AutoRoutine routine = m_autoFactory.newRoutine("DepotToMidLeft");
    AutoTrajectory depotToMidLeft = routine.trajectory("DepotToMidLeft");
    
    DogLog.log("Auto/CurrentPath", depotToMidLeft.active().getAsBoolean());

    routine.active().onTrue(
      Commands.sequence(
        depotToMidLeft.resetOdometry(),
        depotToMidLeft.cmd()));
    return routine;
  }

  public AutoRoutine leftStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToCenter");
    AutoTrajectory leftStartToCenter = routine.trajectory("LeftStartToCenter");
    
    DogLog.log("Auto/CurrentPath", leftStartToCenter.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        leftStartToCenter.resetOdometry(),
        leftStartToCenter.cmd()));
    return routine;
  }
  
  public AutoRoutine leftStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToDepot");
    AutoTrajectory leftStartToDepot = routine.trajectory("LeftStartToDepot");
    
    DogLog.log("Auto/CurrentPath", leftStartToDepot.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        leftStartToDepot.resetOdometry(),
        leftStartToDepot.cmd()));
    return routine;
  }
  
  public AutoRoutine midLeftToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidLeftToDepot");
    AutoTrajectory midLeftToDepot = routine.trajectory("MidLeftToDepot");
    
    DogLog.log("Auto/CurrentPath", midLeftToDepot.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        midLeftToDepot.resetOdometry(),
        midLeftToDepot.cmd()));
    return routine;
  }
  
  public AutoRoutine midRightToOutpost() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidRightToOutpost");
    AutoTrajectory midRightToOutpost = routine.trajectory("MidRightToOutpost");
    
    DogLog.log("Auto/CurrentPath", midRightToOutpost.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        midRightToOutpost.resetOdometry(),
        midRightToOutpost.cmd()));
    return routine;
  }
  
  public AutoRoutine midStartToDepot() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToDepot");
    AutoTrajectory midStartToDepot = routine.trajectory("MidStartToDepot");
    
    DogLog.log("Auto/CurrentPath", midStartToDepot.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        midStartToDepot.resetOdometry(),
        midStartToDepot.cmd()));
    return routine;
  }
  
  public AutoRoutine midStartToLeftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("MidStartToLeftStart");
    AutoTrajectory midStartToLeftStart = routine.trajectory("MidStartToLeftStart");
    
    DogLog.log("Auto/CurrentPath", midStartToLeftStart.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        midStartToLeftStart.resetOdometry(),
        midStartToLeftStart.cmd()));
    return routine;
  }
  
  public AutoRoutine outpostToMidRight() {
    AutoRoutine routine = m_autoFactory.newRoutine("OutpostToMidRight");
    AutoTrajectory outpostToMidRight = routine.trajectory("OutpostToMidRight");
    
    DogLog.log("Auto/CurrentPath", outpostToMidRight.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        outpostToMidRight.resetOdometry(),
        outpostToMidRight.cmd()));
    return routine;
  }
  
  public AutoRoutine rightStartToCenter() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToCenter");
    AutoTrajectory rightStartToCenter = routine.trajectory("RightStartToCenter");
    
    DogLog.log("Auto/CurrentPath", rightStartToCenter.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        rightStartToCenter.resetOdometry(),
        rightStartToCenter.cmd()));
    return routine;
  }

  public AutoRoutine rightToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightToStealBalls");
    AutoTrajectory rightToStealBalls = routine.trajectory("RightToStealBalls");
    
    DogLog.log("Auto/CurrentPath", rightToStealBalls.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        rightToStealBalls.resetOdometry(),
        rightToStealBalls.cmd()));
    return routine;
  }

  public AutoRoutine leftToStealBalls() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftToStealBalls");
    AutoTrajectory leftToStealBalls = routine.trajectory("LeftToStealBalls");
    
    DogLog.log("Auto/CurrentPath", leftToStealBalls.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        leftToStealBalls.resetOdometry(),
        leftToStealBalls.cmd()));
    return routine;

    
  }

  public AutoRoutine leftStartToRightStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("LeftStartToRightStart");
    AutoTrajectory leftStartToRightStart = routine.trajectory("LeftStartToRightStart");
    
    DogLog.log("Auto/CurrentPath", leftStartToRightStart.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        leftStartToRightStart.resetOdometry(),
        leftStartToRightStart.cmd()));
    return routine;

    
  }

  public AutoRoutine rightStartToleftStart() {
    AutoRoutine routine = m_autoFactory.newRoutine("RightStartToleftStart");
    AutoTrajectory rightStartToleftStart = routine.trajectory("RightStartToleftStart");
    
    DogLog.log("Auto/CurrentPath", rightStartToleftStart.active().getAsBoolean());
    
    routine.active().onTrue(
      Commands.sequence(
        rightStartToleftStart.resetOdometry(),
        rightStartToleftStart.cmd()));
    return routine;

    
  }
}

