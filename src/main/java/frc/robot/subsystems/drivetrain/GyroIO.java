package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public Rotation3d rotation3d = new Rotation3d();
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }

  public void resetHeading();

  public Pigeon2SimState getPigeon2SimState();

}
