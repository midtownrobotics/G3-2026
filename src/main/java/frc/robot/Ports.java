package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Ports {
  public record CANPort(int canId, CANBus canbus) {};

  private static final CANBus staticCanBus = new CANBus("staticBus");
  private static final CANBus dynamicCanBus = new CANBus("dynamicBus");

  public static final CANPort kIntakePivot = new CANPort(23, staticCanBus);
  public static final CANPort kIntakeRoller = new CANPort(24, staticCanBus);
  public static final CANPort kIntakePivotEncoder = new CANPort(25, staticCanBus);

  public static final CANPort kTurretHood =  new CANPort(26, staticCanBus);
  public static final CANPort kTurretHoodEncoder = new CANPort(27, staticCanBus);

  public static final CANPort kTurretShooter1 = new CANPort(28, staticCanBus);
  public static final CANPort kTurretShooter2 = new CANPort(29, staticCanBus);

  public static final CANPort kTurretYaw = new CANPort(30, staticCanBus);
  public static final CANPort kTurretYawEncoder1 = new CANPort(31, staticCanBus);
  public static final CANPort kTurretYawEncoder2 = new CANPort(32, staticCanBus);

  public static final CANPort kIndexerTransportRoller = new CANPort(33, staticCanBus);

  public static final CANPort kFeederBelt = new CANPort(34, staticCanBus);
  public static final CANPort kFeederFuelSensor = new CANPort(35, staticCanBus);
}
