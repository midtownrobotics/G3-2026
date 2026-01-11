package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
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
