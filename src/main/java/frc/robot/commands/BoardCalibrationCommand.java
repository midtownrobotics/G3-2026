package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BoardCalibrationPlan.Action;
import frc.robot.commands.BoardCalibrationPlan.Step;
import frc.robot.subsystems.drive.CalibrationDriveLimits;
import frc.robot.subsystems.drive.Drive;

/** Hold-to-run, encoder/gyro-relative calibration itinerary. */
public class BoardCalibrationCommand extends Command {
  private static final String kPlanFile = "board_calibration.json";
  private static final double kDistanceToleranceMeters = 0.03;
  private static final double kAngleToleranceRadians = Math.toRadians(1.5);

  private final Drive m_drive;
  private BoardCalibrationPlan m_plan;
  private Pose2d m_startPose;
  private int m_stepIndex;
  private double m_stepStart;
  private double m_stepTimeout;
  private double m_holdHeading;
  private double m_previousHeading;
  private double m_unwrappedYaw;
  private boolean m_finished;

  public BoardCalibrationCommand(Drive drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.stop();
    m_finished = false;
    m_stepIndex = 0;
    m_startPose = m_drive.getPose();
    m_previousHeading = m_startPose.getRotation().getRadians();
    m_unwrappedYaw = 0;
    Logger.recordOutput("Calibration/abortReason", "");
    Logger.recordOutput("Calibration/startPose", m_startPose);

    if (!m_drive.isGyroConnected()) {
      abort("gyro disconnected");
      return;
    }
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve(kPlanFile);
      m_plan = BoardCalibrationPlan.load(path);
      Logger.recordOutput("Calibration/plan", m_plan.toString());
      beginStep();
    } catch (IOException | IllegalArgumentException ex) {
      abort("invalid " + kPlanFile + ": " + ex.getMessage());
    }
  }

  private void beginStep() {
    m_drive.stop();
    if (m_stepIndex == m_plan.steps().size()) {
      m_finished = true;
      return;
    }
    m_holdHeading = m_unwrappedYaw;
    m_stepStart = Timer.getFPGATimestamp();
    Step step = m_plan.steps().get(m_stepIndex);
    if (step.action() == Action.MOVE) {
      double cos = m_startPose.getRotation().getCos();
      double sin = m_startPose.getRotation().getSin();
      Pose2d pose = m_drive.getPose();
      double dx = m_startPose.getX() - step.awayMeters() * cos - step.leftMeters() * sin - pose.getX();
      double dy = m_startPose.getY() - step.awayMeters() * sin + step.leftMeters() * cos - pose.getY();
      m_stepTimeout = Math.min(45, Math.hypot(dx, dy) / CalibrationDriveLimits.kMaxLinearMetersPerSecond + 20);
    } else if (step.action() == Action.ROTATE) {
      m_stepTimeout = Math.min(150,
          Math.abs(Math.toRadians(step.headingDegrees()) - m_unwrappedYaw)
              / CalibrationDriveLimits.kMaxAngularRadiansPerSecond + 20);
    }
  }

  private void abort(String reason) {
    DriverStation.reportError("Board calibration stopped: " + reason, false);
    Logger.recordOutput("Calibration/abortReason", reason);
    m_drive.stop();
    m_finished = true;
  }

  @Override
  public void execute() {
    if (m_finished) {
      return;
    }
    Pose2d pose = m_drive.getPose();
    double heading = pose.getRotation().getRadians();
    m_unwrappedYaw += MathUtil.angleModulus(heading - m_previousHeading);
    m_previousHeading = heading;
    Step step = m_plan.steps().get(m_stepIndex);
    double elapsed = Timer.getFPGATimestamp() - m_stepStart;

    Logger.recordOutput("Calibration/stepIndex", m_stepIndex);
    Logger.recordOutput("Calibration/phase", step.action().name());
    Logger.recordOutput("Calibration/capturing", step.action() == Action.WAIT);
    Logger.recordOutput("Calibration/unwrappedYawRadians", m_unwrappedYaw);
    Logger.recordOutput("Calibration/targetAwayMeters", step.awayMeters());
    Logger.recordOutput("Calibration/targetLeftMeters", step.leftMeters());
    Logger.recordOutput("Calibration/targetHeadingDegrees", step.headingDegrees());

    if (!m_drive.isGyroConnected()) {
      abort("gyro disconnected");
      return;
    }

    switch (step.action()) {
      case MOVE -> {
        double cos = m_startPose.getRotation().getCos();
        double sin = m_startPose.getRotation().getSin();
        double targetX = m_startPose.getX() - step.awayMeters() * cos - step.leftMeters() * sin;
        double targetY = m_startPose.getY() - step.awayMeters() * sin + step.leftMeters() * cos;
        double errorX = targetX - pose.getX();
        double errorY = targetY - pose.getY();
        double distance = Math.hypot(errorX, errorY);
        if (distance <= kDistanceToleranceMeters) {
          nextStep();
        } else if (elapsed > m_stepTimeout) {
          abort("move timeout at step " + m_stepIndex);
        } else {
          double speed = Math.min(CalibrationDriveLimits.kMaxLinearMetersPerSecond, 1.5 * distance);
          double turn = MathUtil.clamp(2 * MathUtil.angleModulus(m_holdHeading - m_unwrappedYaw),
              -CalibrationDriveLimits.kMaxAngularRadiansPerSecond, CalibrationDriveLimits.kMaxAngularRadiansPerSecond);
          m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
              speed * errorX / distance, speed * errorY / distance, turn, pose.getRotation()));
        }
      }
      case ROTATE -> {
        double error = Math.toRadians(step.headingDegrees()) - m_unwrappedYaw;
        if (Math.abs(error) <= kAngleToleranceRadians) {
          nextStep();
        } else if (elapsed > m_stepTimeout) {
          abort("rotation timeout at step " + m_stepIndex);
        } else {
          m_drive.runVelocity(new ChassisSpeeds(0, 0,
              MathUtil.clamp(1.5 * error, -CalibrationDriveLimits.kMaxAngularRadiansPerSecond, CalibrationDriveLimits.kMaxAngularRadiansPerSecond)));
        }
      }
      case WAIT -> {
        m_drive.stop();
        if (elapsed >= step.seconds()) {
          nextStep();
        }
      }
    }
  }

  private void nextStep() {
    m_stepIndex++;
    beginStep();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    Logger.recordOutput("Calibration/capturing", false);
    Logger.recordOutput("Calibration/phase", interrupted ? "INTERRUPTED" : "DONE");
  }
}
