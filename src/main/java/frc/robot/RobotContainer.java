package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  public RobotContainer() {

  }

  public void periodic() {

  }

  public void teleopInit() {

  }

  public Command getAutonomousCommand() {
    return new RunCommand(() -> {
    });
  }

}
