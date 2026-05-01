package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
  // (50/12) * (60/20) * (48/16) = 37.5
  private static final double kGearRatio = (50.0 / 12.0) * (60.0 / 20.0) * (48.0 / 16.0);
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
  // Approximate arm length (meters) and mass (kg)
  private static final double ARM_LENGTH = 0.4;
  private static final double ARM_MASS = 3.0;
  // Approximate MOI (kg*m^2)
  private static final double MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS);

  private final SingleJointedArmSim m_sim;
  private final ProfiledPIDController m_controller;

  private double m_appliedVolts = 0.0;
  private boolean m_closedLoop = false;
  private double m_kG = 0.5;
  private Angle m_setpoint = Degrees.zero();

  public IntakePivotIOSim() {
    m_sim = new SingleJointedArmSim(
        MOTOR,
        kGearRatio,
        MOI,
        ARM_LENGTH,
        Math.toRadians(-10), // min angle
        Math.toRadians(60), // max angle
        true, // simulate gravity
        Math.toRadians(0)); // starting angle

    // MotionMagic equivalent: cruise velocity 180 deg/s, accel 1000 deg/s^2
    m_controller = new ProfiledPIDController(
        3.0,
        0,
        0,
        new TrapezoidProfile.Constraints(
            Math.toRadians(180), // max velocity rad/s
            Math.toRadians(1000) // max accel rad/s^2
        ));
    m_controller.setTolerance(Math.toRadians(1));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (m_closedLoop) {
      double feedforward = Math.cos(m_sim.getAngleRads()) * m_kG; // gravity compensation
      m_appliedVolts = MathUtil.clamp(m_controller.calculate(m_sim.getAngleRads()) + feedforward, -12.0, 12.0);
    }

    m_sim.setInputVoltage(m_appliedVolts);
    m_sim.update(0.02);

    inputs.position = Radians.of(m_sim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(m_sim.getVelocityRadPerSec());
    inputs.appliedVoltage = Volts.of(m_appliedVolts);
    inputs.statorCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.encoderAbsolutePosition = Radians.of(m_sim.getAngleRads());
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = true;
  }

  @Override
  public void setPosition(Angle angle) {
    m_closedLoop = true;
    m_setpoint = angle;
    m_controller.setGoal(angle.in(Radians));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_closedLoop = false;
    m_appliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
  }

  @Override
  public void stop() {
    m_closedLoop = false;
    m_appliedVolts = 0.0;
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    // In sim, we reset the sim state
    m_sim.setState(angle.in(Radians), 0);
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kG) {
    m_controller.setPID(kP, kI, kD);
    m_kG = kG;
  }
}
