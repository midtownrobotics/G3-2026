package frc.robot.utils;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class PhoenixUtil {
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }
}
