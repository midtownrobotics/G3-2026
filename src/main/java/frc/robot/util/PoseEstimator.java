// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import lombok.Getter;

/**
 * Custom pose estimator that fuses swerve odometry with vision observations
 * using a Kalman filter approach. Maintains both a raw odometry pose and a
 * vision-corrected estimated pose.
 */
public class PoseEstimator {
  private static final double kPoseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> kOdometryStdDevs = new Matrix<>(VecBuilder.fill(1, 1, 0.2));

  @Getter
  private Pose2d odometryPose = Pose2d.kZero;
  @Getter
  private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(kPoseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private final LoggedNetworkBoolean m_tiltCompensationEnabled = new LoggedNetworkBoolean(
      "Toggles/OdometryTiltCompensation", false);

	private static LoggedNetworkBoolean m_wallClampEnabled = new LoggedNetworkBoolean(
      "Toggles/WallClamp", true);

  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private Rotation2d gyroOffset = Rotation2d.kZero;

  public PoseEstimator(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(kOdometryStdDevs.get(i, 0), 2));
    }
  }

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(Rotation2d gyroRotation, SwerveModulePosition[] wheelPositions, Pose2d pose) {
    gyroOffset = pose.getRotation().minus(gyroRotation);
    estimatedPose = pose;
    odometryPose = pose;
    lastWheelPositions = wheelPositions;
    poseBuffer.clear();
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  /** Adds a new odometry observation from the drive subsystem. */
  public void addOdometryObservation(OdometryObservation observation) {
    // Scale down odometry when the robot is tilted (e.g. driving over a ramp).
    // Wheel travel on an incline doesn't translate 1:1 to field-plane movement.
    double tiltScale = 1.0;

    if (m_tiltCompensationEnabled.get() && observation.pitch().isPresent() && observation.roll().isPresent()) {
      double cosProduct = observation.pitch().get().getCos()
          * observation.roll().get().getCos();
      double tiltDegrees = Math.abs(Math.toDegrees(Math.acos(cosProduct)));
      tiltScale = MathUtil.clamp(1.0 - MathUtil.inverseInterpolate(0, 25, tiltDegrees), 0.0, 1.0);
    }

    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    twist = new Twist2d(twist.dx * tiltScale, twist.dy * tiltScale, twist.dtheta * tiltScale);
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    // Replace odometry heading with gyro if present
    observation.yaw().ifPresent(gyroAngle -> {
      Rotation2d angle = gyroAngle.plus(gyroOffset);
      odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
    });

    // Clamp the odometry pose to the field boundaries to prevent large errors from accumulating
    // odometryPose = clampPose2dToFieldBounds(odometryPose);
		Logger.recordOutput("PoseEstimator/odometryPose", odometryPose);

    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);

    // Apply odometry delta to the vision-corrected estimated pose
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);

		if (m_wallClampEnabled.get()) {
			estimatedPose = clampPose2dToFieldBounds(estimatedPose);
		}
  }

  private static Pose2d clampPose2dToFieldBounds(Pose2d pose) {
    // The axis-aligned bounding box half-extents of the rotated robot rectangle.
    // For a rectangle of dimensions L x W rotated by θ:
    //   halfExtentX = (|L*cosθ| + |W*sinθ|) / 2
    //   halfExtentY = (|L*sinθ| + |W*cosθ|) / 2

		// actual code vvvvvv

    double cosTheta = Math.abs(pose.getRotation().getCos());
    double sinTheta = Math.abs(pose.getRotation().getSin());
    double robotLength = Constants.kRobotLengthWithBumpers.in(Meters);
    double robotWidth = Constants.kRobotWidthWithBumpers.in(Meters);

    double offsetX = (robotLength * cosTheta + robotWidth * sinTheta) / 2.0;
    double offsetY = (robotLength * sinTheta + robotWidth * cosTheta) / 2.0;

		 
			return new Pose2d(
        new Translation2d(
            MathUtil.clamp(pose.getX(), offsetX, FieldConstants.kFieldLength.in(Meters) - offsetX),
            MathUtil.clamp(pose.getY(), offsetY, FieldConstants.kFieldWidth.in(Meters) - offsetY)),
        pose.getRotation());
  }

  /** Adds a new vision pose observation from the vision subsystem. */
  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - kPoseBufferSizeSec > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      return;
    }

    // Calculate transforms between odometry pose at sample time and current odometry pose
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

    // Shift estimated pose backwards to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3x3 vision covariance
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Calculate the transform from the shifted estimate to the observation pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

    // Scale the transform by the Kalman gain
    var kTimesTransform = visionK.times(
        VecBuilder.fill(
            transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform = new Transform2d(
        kTimesTransform.get(0, 0),
        kTimesTransform.get(1, 0),
        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate the current estimate by applying the scaled transform at sample time,
    // then shifting forward to the present using odometry data
		estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);

    if (m_wallClampEnabled.get()) {
			estimatedPose = clampPose2dToFieldBounds(estimatedPose);
		}
  }

  /**
   * Get the estimated pose at a past timestamp by projecting the current estimate
   * backward using the odometry buffer.
   */
  public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
    var oldOdometryPose = poseBuffer.getSample(timestamp);
    if (oldOdometryPose.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        getEstimatedPose()
            .transformBy(new Transform2d(getOdometryPose(), oldOdometryPose.get())));
  }

  // MARK: - Record types

  public record OdometryObservation(
      double timestamp,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> pitch,
      Optional<Rotation2d> roll,
      Optional<Rotation2d> yaw) {
  }

  public record VisionObservation(
      double timestamp,
      Pose3d visionPose,
      Matrix<N3, N1> stdDevs) {
  }
}
