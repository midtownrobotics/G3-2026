package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
  private final RobotState m_state;
  private final ShootingParameters m_parameters;

  public Dashboard(RobotState state, ShootingParameters parameters) {
    m_state = state;
    m_parameters = parameters;
  }

  public void periodic() {
    SmartDashboard.putBoolean("ShootingParameters/hasShot",
        m_parameters.getLockOutStatus() == ShootingParameters.LockOutStatus.kNotLockedOut);

    SmartDashboard.putBoolean("ShootingParameters/turretWithinTolerance",
        m_parameters.getLockOutStatus() != ShootingParameters.LockOutStatus.kTurretNotWithinTolerance);
    SmartDashboard.putBoolean("ShootingParameters/hoodWithinTolerance",
        m_parameters.getLockOutStatus() != ShootingParameters.LockOutStatus.kHoodNotWithinTolerance);
    SmartDashboard.putBoolean("ShootingParameters/flywheelWithinTolerance",
        m_parameters.getLockOutStatus() != ShootingParameters.LockOutStatus.kFlywheelNotWithinTolerance);
  }
}
