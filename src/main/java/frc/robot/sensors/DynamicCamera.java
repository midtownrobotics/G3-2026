package frc.robot.sensors;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Transform3d;

public class DynamicCamera extends Camera {
  private Supplier<Transform3d> m_robotToCameraSupplier;

  public DynamicCamera(String name, double stdDevMultiplier) {
    super(name, new Transform3d(), stdDevMultiplier);
  }

  @Override
  public void periodic() {
    super.periodic();
    m_robotToCamera = m_robotToCameraSupplier.get();
  }

  public void addRobotToCameraSupplier(Supplier<Transform3d> robotToCameraSupplier) {
    m_robotToCameraSupplier = robotToCameraSupplier;
  }
}
