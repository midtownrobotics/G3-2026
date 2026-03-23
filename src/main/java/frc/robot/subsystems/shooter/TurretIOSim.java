package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private static final double kGearRatio = 48.0;
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
  // Approximate MOI for a turret (kg*m^2)
  private static final double MOI = 0.05;

  private final DCMotorSim m_sim;
  private final PIDController m_controller = new PIDController(1.5, 0, 0);

  private double m_appliedVolts = 0.0;
  private boolean m_closedLoop = false;
  private Angle m_setpoint = Degrees.zero();

  public TurretIOSim() {
    m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, MOI, kGearRatio), MOTOR);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (m_closedLoop) {
      m_appliedVolts = MathUtil.clamp(
          m_controller.calculate(
              m_sim.getAngularPositionRad(), m_setpoint.in(Radians)),
          -12.0,
          12.0);
    }

    m_sim.setInputVoltage(m_appliedVolts);
    m_sim.update(0.02);

    inputs.position = Rotations.of(m_sim.getAngularPositionRotations());
    inputs.velocity = RPM.of(m_sim.getAngularVelocityRPM());
    inputs.appliedVoltage = Volts.of(m_appliedVolts);
    inputs.statorCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.encoder1AbsolutePosition = Rotations.of(m_sim.getAngularPositionRotations());
    inputs.encoder2AbsolutePosition = Rotations.of(m_sim.getAngularPositionRotations());
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = true;
  }

  @Override
  public void setPosition(Angle angle) {
    m_closedLoop = true;
    m_setpoint = angle;
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
    m_sim.setState(angle.in(Rotations) * kGearRatio * 2 * Math.PI, 0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_controller.setPID(kP, kI, kD);
  }
}
