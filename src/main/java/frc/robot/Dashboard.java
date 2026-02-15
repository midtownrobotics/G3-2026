package frc.robot;

public class Dashboard {
  private final RobotState m_state;
  private final ShootingParameters m_parameters;

  public Dashboard(RobotState state, ShootingParameters parameters) {
    m_state = state;
    m_parameters = parameters;
  }
}
