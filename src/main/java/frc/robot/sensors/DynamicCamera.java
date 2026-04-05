package frc.robot.sensors;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class DynamicCamera extends Camera {
  private Supplier<Transform3d> m_robotToCameraSupplier;

  public DynamicCamera(String name, Matrix<N3, N1> standardDevs) {
    super(name, new Transform3d(), standardDevs);
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
