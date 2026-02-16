package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
  private final RobotState m_state;
  private final ShootingParameters m_parameters;
  private final Field2d m_field = new Field2d();

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

    SmartDashboard.putBoolean("GameData/isHubActive", isHubActive());

    m_field.setRobotPose(m_state.getRobotPose());

    SmartDashboard.putData("Field", m_field);
  }

  private boolean isHubActive() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty())
      return false;

    if (DriverStation.isAutonomousEnabled())
      return true;
    if (!DriverStation.isTeleopEnabled())
      return false;

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.isEmpty())
      return true;

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active = (alliance.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130)
      return true;
    if (matchTime > 105)
      return shift1Active;
    if (matchTime > 80)
      return !shift1Active;
    if (matchTime > 55)
      return shift1Active;
    if (matchTime > 30)
      return !shift1Active;
    return true;
  }
}
