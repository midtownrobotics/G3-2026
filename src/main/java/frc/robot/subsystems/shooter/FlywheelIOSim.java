package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private static final double kGearRatio = 2.0 / 3.0;
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
  // Approximate MOI for a flywheel (kg*m^2)
  private static final double MOI = 0.004;

  private final DCMotorSim m_sim;
  private final PIDController m_controller = new PIDController(0.05, 0, 0);

  private double m_appliedVolts = 0.0;
  private boolean m_closedLoop = false;
  private AngularVelocity m_setpoint = RPM.zero();

  public FlywheelIOSim() {
    m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, MOI, kGearRatio), MOTOR);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (m_closedLoop) {
      m_appliedVolts = MathUtil.clamp(
          m_controller.calculate(
              m_sim.getAngularVelocityRPM(), m_setpoint.in(RPM)),
          -12.0,
          12.0);
    }

    m_sim.setInputVoltage(m_appliedVolts);
    m_sim.update(0.02);

    inputs.velocity = RPM.of(m_sim.getAngularVelocityRPM());
    inputs.appliedVoltage = Volts.of(m_appliedVolts);
    inputs.statorCurrent1 = Amps.of(m_sim.getCurrentDrawAmps() / 2.0);
    inputs.statorCurrent2 = Amps.of(m_sim.getCurrentDrawAmps() / 2.0);
    inputs.supplyCurrent1 = Amps.of(m_sim.getCurrentDrawAmps() / 2.0);
    inputs.supplyCurrent2 = Amps.of(m_sim.getCurrentDrawAmps() / 2.0);
    inputs.velocity1 = RPM.of(m_sim.getAngularVelocityRPM());
    inputs.velocity2 = RPM.of(m_sim.getAngularVelocityRPM());
    inputs.setpoint = m_setpoint;
    inputs.motor1Connected = true;
    inputs.motor2Connected = true;
  }

  @Override
  public void setSpeed(AngularVelocity speed) {
    m_closedLoop = true;
    m_setpoint = speed;
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
  public void setPID(double kP, double kI, double kD, double kS, double kV) {
    m_controller.setPID(kP, kI, kD);
    // kS and kV are feedforward terms, not directly applicable to sim PIDController
  }
}
