package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class Ports {
  public record CANPort(int canId, CANBus canbus) {
  };

  public static final CANBus primaryCanBus = new CANBus("mechs");
  public static final CANBus driveCanBus = new CANBus("drive");

  public static final CANPort kIntakePivot = new CANPort(23, primaryCanBus);
  public static final CANPort kIntakeRoller = new CANPort(24, primaryCanBus);
  public static final CANPort kIntakePivotEncoder = new CANPort(25, primaryCanBus);

  public static final CANPort kTurretHood = new CANPort(26, primaryCanBus);
  public static final CANPort kTurretHoodEncoder = new CANPort(27, primaryCanBus);

  public static final CANPort kTurretShooter1 = new CANPort(28, primaryCanBus);
  public static final CANPort kTurretShooter2 = new CANPort(29, primaryCanBus);

  public static final CANPort kTurretYaw = new CANPort(30, primaryCanBus);
  public static final CANPort kTurretYawEncoder1 = new CANPort(31, primaryCanBus);
  // public static final CANPort kTurretYawEncoder2 = new CANPort(32, primaryCanBus);

  public static final CANPort kIndexer = new CANPort(33, primaryCanBus);

  public static final CANPort kFeederBeltLeader = new CANPort(34, primaryCanBus);
  // public static final CANPort kFeederFuelSensor = new CANPort(35, primaryCanBus);
  public static final CANPort kFeederBeltFollower = new CANPort(36, primaryCanBus);
}
