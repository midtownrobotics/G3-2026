package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Current KRAKEN_CURRENT_LIMIT = Units.Amp.of(70);
  public static final Current KRAKEN_CURRENT_LOWER_LIMIT = Units.Amp.of(40);
  public static final Current NEO_550_CURRENT_LIMIT = Units.Amp.of(25);
  public static final Current NEO_CURRENT_LIMIT = Units.Amp.of(60);
  public static final Current BAG_CURRENT_LIMIT = Units.Amp.of(70);

  public static final LoggedNetworkBoolean tuningMode = new LoggedNetworkBoolean("TuningMode", false);
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode MODE;
  public static final boolean enableReplay = false;

  static {
    if (RobotBase.isReal())
      MODE = Mode.REAL;
    else if (enableReplay)
      MODE = Mode.REPLAY;
    else
      MODE = Mode.SIM;
  }
}
