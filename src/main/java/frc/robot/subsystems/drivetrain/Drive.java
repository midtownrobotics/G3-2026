package frc.robot.subsystems.drivetrain;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  public static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName)
      .isNetworkFD()
          ? 250.0
          : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private static final double ROBOT_MASS_KG = 66.3;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
}
