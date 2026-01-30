package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;
  public final IntakePivot m_intakePivot;
  public final IntakeRoller m_intakeRoller;

  public RobotState(Controls controls, CommandSwerveDrivetrain drive, IntakePivot intakePivot,
      IntakeRoller intakeRoller) {
    m_controls = controls;
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;

    m_drive.setDefaultCommand(DriveCommands.joyStickDrive(drive, controls));
    m_controls.snakeDrive().whileTrue(DriveCommands.snakeDrive(drive, controls));
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public Trigger inAllianceZone() {
    return new Trigger(
        () -> DriverStation.getAlliance()
            .map(FieldConstants::getAllianceZone)
            .map(r -> r.contains(m_drive.getPose().getTranslation()))
            .orElse(false))
        .debounce(0.2);
  }

  public Angle getIntakeAngle() {
    return m_intakePivot.getAngle();
  }

}
