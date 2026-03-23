package frc.robot.subsystems.indexer;

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

public class IndexerIOSim implements InexerIO {
  private static final double kGearRatio = 20.0 / 14.0;
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
  private static final double MOI = 0.001;

  private final DCMotorSim m_sim;
  private final PIDController m_controller = new PIDController(0.1, 0, 0);

  private double m_appliedVolts = 0.0;
  private boolean m_closedLoop = false;
  private AngularVelocity m_setpoint = RPM.zero();

  public IndexerIOSim() {
    m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, MOI, kGearRatio), MOTOR);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
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
    inputs.statorCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.setpoint = m_setpoint;
    inputs.motorConnected = true;
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
  public void setPID(double kP, double kI, double kD) {
    m_controller.setPID(kP, kI, kD);
  }
}
