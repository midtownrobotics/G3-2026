package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drive;

public class Autos {
  private final AutoFactory m_factory;
  private final Drive m_drive;

  public Autos(Drive drive) {
    m_drive = drive;
    m_factory = new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
  }

  public AutoChooser createAutoChooser() {
    AutoChooser chooser = new AutoChooser();
    chooser.addRoutine("My Auto", this::myAuto);
    return chooser;
  }

  public AutoRoutine myAuto() {
    AutoRoutine routine = m_factory.newRoutine("My Auto");

    AutoTrajectory testPath = routine.trajectory("TestPath");

    routine.active().onTrue(Commands.sequence(
        testPath.resetOdometry(),
        testPath.cmd()));

    return routine;
  }
}
