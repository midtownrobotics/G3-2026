package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PoseEstimator;
import frc.robot.util.PoseEstimator.OdometryObservation;
import frc.robot.util.PoseEstimator.VisionObservation;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(
              TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(
              TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  /** Minimum translational speed (m/s) below which skid ratio is not computed (noise-dominated). */
  private static final double kSkidMinSpeedMetersPerSec = 0.1;
  /** Skid ratio threshold above which we consider the robot to be skidding. */
  private static final double kSkidRatioThreshold = 1.5;
  /** Duration (seconds) after a skid event during which vision trust is boosted. */
  private static final double kSkidVisionBoostDurationSec = 2.5;
  /** Factor by which odometry std devs are inflated during the post-skid boost window. */
  private static final double kSkidOdometryStdDevMultiplier = 3.0;

  /** XY acceleration magnitude (m/s^2) above which we consider a collision has occurred. 2g ≈ 19.62 m/s^2. */
  private static final double kCollisionThresholdMetersPerSecSq = 2.0 * 9.81;
  /** Duration (seconds) after a collision during which hasCollision() returns true. */
  private static final double kCollisionCooldownSec = 0.5;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final SysIdRoutine sysIdRotation;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private PoseEstimator m_poseEstimator = new PoseEstimator(kinematics);

  private double m_skidRatio = 1.0;
  private double m_lastSkidTimestamp = -100.0; // large negative so boost is inactive at startup
  private double m_lastCollisionTimestamp = -100.0;

  /** PID controllers for Choreo path following */
  private final PIDController m_pathXController = new PIDController(7, 0, 0);
  private final PIDController m_pathYController = new PIDController(7, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(0, 0, 0);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("Drive/Translation/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    sysIdRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(6),
            null,
            (state) -> SignalLogger.writeString("Drive/Rotation/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterizationRotation(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Compute skid ratio using Orbit's method (based on latest module states)
    m_skidRatio = calculateSkiddingRatio(getModuleStates());
    if (isSkidding()) {
      m_lastSkidTimestamp = Logger.getTimestamp() / 1e6; // convert microseconds to seconds
    }

    // Inflate odometry std devs during skid events so the Kalman filter trusts vision more
    m_poseEstimator.setOdometryStdDevMultiplier(
        isSkidVisionBoostActive() ? kSkidOdometryStdDevMultiplier : 1.0);

    // Collision detection from accelerometer
    if (gyroInputs.connected) {
      double xyAccel = Math.hypot(gyroInputs.xAcceleration, gyroInputs.yAcceleration);
      if (xyAccel >= kCollisionThresholdMetersPerSecSq) {
        m_lastCollisionTimestamp = Logger.getTimestamp() / 1e6;
      }
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      var odometryObservation = new OdometryObservation(
          sampleTimestamps[i], modulePositions, gyroInputs.connected
              ? Optional.of(gyroInputs.odometryRollPositions[i])
              : Optional.empty(),
          gyroInputs.connected
              ? Optional.of(gyroInputs.odometryPitchPositions[i])
              : Optional.empty(),
          gyroInputs.connected
              ? Optional.of(gyroInputs.odometryYawPositions[i])
              : Optional.empty());

      // Apply update
      m_poseEstimator.addOdometryObservation(odometryObservation);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && !DriverStation.isTest());

    // Log drive state
    Logger.recordOutput("Drive/pose", getPose());
    Logger.recordOutput("Drive/chassisSpeeds", getChassisSpeeds());
    Logger.recordOutput("Drive/moduleStates", getModuleStates());
    Logger.recordOutput("Drive/modulePositions", getModulePositions());
    Logger.recordOutput("Drive/skidRatio", getSkidRatio());
    Logger.recordOutput("Drive/isSkidding", isSkidding());
    Logger.recordOutput("Drive/skidVisionBoostActive", isSkidVisionBoostActive());
    Logger.recordOutput("Drive/hasCollision", hasCollision());
    Logger.recordOutput("Drive/xyAcceleration",
        Math.hypot(gyroInputs.xAcceleration, gyroInputs.yAcceleration));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/moduleSetpointStates", setpointStates);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Follows the given field-centric Choreo path sample with PID.
   *
   * @param sample Sample along the path to follow
   */
  public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var pose = getPose();
    var targetPose = sample.getPose();

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(pose.getRotation().getRadians(),
        sample.heading);

    Logger.recordOutput("Drive/PathFollower/TargetPose", targetPose);
    Logger.recordOutput("Drive/PathFollower/Error/X", targetPose.getMeasureX().minus(pose.getMeasureX()));
    Logger.recordOutput("Drive/PathFollower/Error/Y", targetPose.getMeasureY().minus(pose.getMeasureY()));
    Logger.recordOutput("Drive/PathFollower/Error/Rotation",
        targetPose.getRotation().minus(pose.getRotation()).getMeasure());

    // Convert field-relative speeds to robot-relative and run
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, pose.getRotation());
    runVelocity(robotRelativeSpeeds);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void runCharacterizationRotation(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterizationRotation(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterizationRotation(0.0))
        .withTimeout(1.0)
        .andThen(sysIdRotation.quasistatic(direction));
  }

  public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterizationRotation(0.0)).withTimeout(1.0).andThen(sysIdRotation.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPose(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry pose. Alias for setPose for compatibility. */
  public void resetPose(Pose2d pose) {
    setPose(pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose3d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    var visionObservation = new VisionObservation(timestampSeconds, visionRobotPoseMeters, visionMeasurementStdDevs);
    m_poseEstimator.addVisionObservation(visionObservation);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public double getSkidRatio() {
    return m_skidRatio;
  }

  /** Returns whether the robot is currently skidding based on the Orbit skid ratio. */
  public boolean isSkidding() {
    return getSkidRatio() > kSkidRatioThreshold;
  }

  /** Returns true for kCollisionCooldownSec after any accelerometer reading exceeds 2g. */
  public boolean hasCollision() {
    double now = Logger.getTimestamp() / 1e6;
    return (now - m_lastCollisionTimestamp) < kCollisionCooldownSec;
  }

  /** Returns whether we are within the post-skid window where vision trust is boosted. */
  public boolean isSkidVisionBoostActive() {
    double now = Logger.getTimestamp() / 1e6;
    return (now - m_lastSkidTimestamp) < kSkidVisionBoostDurationSec;
  }

  /**
   * Calculates the skidding ratio using 1690 Orbit's method.
   *
   * <p>The idea: decompose each module's measured velocity vector into a rotational component
   * (from chassis ω) and a translational component (the remainder). If no wheel is skidding,
   * all translational components should have the same magnitude. The ratio of max/min
   * translational magnitudes indicates how much disagreement there is — a ratio significantly
   * above 1.0 means at least one wheel is slipping.
   * 
   * https://www.chiefdelphi.com/t/has-anyone-successfully-implemented-orbits-odometry-skid-detection/468257
   *
   * @param measuredStates the current measured swerve module states
   * @return the skidding ratio, in the range [1, ∞). Returns 1.0 if speeds are too low to measure.
   */
  private double calculateSkiddingRatio(SwerveModuleState[] measuredStates) {
    // Extract the chassis angular velocity from the measured module states
    double omega = kinematics.toChassisSpeeds(measuredStates).omegaRadiansPerSecond;

    // Compute what each module's state would be for pure rotation at that ω
    SwerveModuleState[] rotationalStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, omega));

    // For each module, vector-subtract the rotational component to isolate translation
    double maxTranslational = 0.0;
    double minTranslational = Double.POSITIVE_INFINITY;
    for (int i = 0; i < measuredStates.length; i++) {
      Translation2d measuredVec = new Translation2d(
          measuredStates[i].speedMetersPerSecond, measuredStates[i].angle);
      Translation2d rotationalVec = new Translation2d(
          rotationalStates[i].speedMetersPerSecond, rotationalStates[i].angle);
      double translationalMagnitude = measuredVec.minus(rotationalVec).getNorm();

      maxTranslational = Math.max(maxTranslational, translationalMagnitude);
      minTranslational = Math.min(minTranslational, translationalMagnitude);
    }

    // Guard against division by zero / noise at low speeds
    if (maxTranslational < kSkidMinSpeedMetersPerSec) {
      return 1.0;
    }

    return maxTranslational / Math.max(minTranslational, 1e-6);
  }

  /** Returns the swerve drive kinematics. */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
