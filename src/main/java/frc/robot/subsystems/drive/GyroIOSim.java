package frc.robot.subsystems.drive;

public class GyroIOSim implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
  }
}
