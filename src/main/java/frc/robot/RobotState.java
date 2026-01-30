package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class RobotState {
  public final Controls m_controls;
  public final CommandSwerveDrivetrain m_drive;

  public RobotState(Controls controls, CommandSwerveDrivetrain drive) {
    m_controls = controls;
    m_drive = drive;

    m_drive.setDefaultCommand(DriveCommands.intakeDrive(drive, controls));
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
}
