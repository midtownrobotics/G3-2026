package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private static final double kGearRatio = 1.0;
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
  private static final double MOI = 0.001;

  private final DCMotorSim m_sim;
  private double m_appliedVolts = 0.0;
  private Voltage m_setpoint = Volts.zero();

  public IntakeRollerIOSim() {
    m_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, MOI, kGearRatio), MOTOR);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
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
  public void setVoltage(Voltage voltage) {
    m_appliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    m_setpoint = voltage;
  }

  @Override
  public void stop() {
    m_appliedVolts = 0.0;
    m_setpoint = Volts.zero();
  }
}
