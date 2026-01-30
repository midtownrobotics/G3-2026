package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class AutoAim {

  public static Command getAutoAimCommand(RobotState state) {

    Supplier<Angle> turretAngle = () -> {
      Pose2d pose = state.getRobotPose().transformBy(new Transform2d(new Translation2d(-0.1, 0.2), new Rotation2d()));
      Translation2d hubTranslation = new Translation2d(Meters.of(11.7), Meters.of(4.2));
      return hubTranslation.minus(pose.getTranslation()).getAngle().minus(pose.getRotation()).getMeasure();
    };

    return state.getTurret().setYawAngleCommand(turretAngle);
  }
}
