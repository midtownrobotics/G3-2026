package frc.robot;

import com.ctre.phoenix6.CANBus;

public class Ports {
  public record CANPort(int canId, CANBus canbus) {};

  private static final CANBus staticCanBus = new CANBus("staticBus");
  private static final CANBus dynamicCanBus = new CANBus("dynamicBus");

  public static final CANPort kFeederBeltTalonFXPort = new CANPort(34, staticCanBus);
  public static final CANPort kFeederFuelSensor = new CANPort(35, staticCanBus);

  public static final CANPort kIntakePivotTalonFXPort = new CANPort(23, staticCanBus);
  public static final CANPort kIntakeRollerTalonFXPort = new CANPort(24, staticCanBus);
  public static final CANPort kIntakePivotCANPort = new CANPort(25, staticCanBus);

  public static final CANPort kTurretHoodTalonFXPort = new CANPort(26, staticCanBus);
  public static final CANPort kTurretShooterMotorTalonFXPort = new CANPort(28, staticCanBus);
  public static final CANPort kTurretYawMotorTalonFXPort = new CANPort(30, staticCanBus);
  public static final CANPort kTurretCANCoder1Port = new CANPort(31, staticCanBus);
  public static final CANPort kTurretCANCoder2Port = new CANPort(32, staticCanBus);

  public static final CANPort kIndexerTransportRollerTalonFXPort = new CANPort(33, staticCanBus);
}
