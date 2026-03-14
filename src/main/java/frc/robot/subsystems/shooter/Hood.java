package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Logger;
import frc.lib.PhoenixUtil;
import frc.lib.Watchdawg;
import frc.robot.constants.Ports;

public class Hood extends SubsystemBase{
    private static final double kMechanismToMotorGearing = 266d;
    private static final double kMechanismToEncoderGearing = 19d;
    private static final Angle kLowSoftLimit = Degrees.of(0);
    private static final Angle kHighSoftLimit = Degrees.of(55);


    private final CANcoder m_encoder;
    private final Logger m_log;
    private final Trigger m_currentSpikeTrigger;
    private final LinearFilter m_currentSpikeFilter;
    private final Alert m_talonConnectionAlert = new Alert("Hood motor controller is not connected", AlertType.kWarning);
    private final TalonFX m_motor;
    private final Watchdawg m_watchdog;

    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    public Hood() {
        m_motor = new TalonFX(Ports.kTurretHood.canId(), Ports.kTurretHood.canbus());
        m_encoder = new CANcoder(Ports.kTurretHoodEncoder.canId(), Ports.kTurretHoodEncoder.canbus());
        m_log = new Logger(getClass());
        m_watchdog = new Watchdawg(getClass());
        m_currentSpikeFilter = LinearFilter.movingAverage(5);
        m_currentSpikeTrigger = new Trigger(this::getIsCurrentSpiking);
        configureMotor();
        seedEncoderPosition();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = new Slot0Configs()
            .withKP(250)
            .withKI(0)
            .withKD(0)
            .withKS(0.01);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.withMotionMagicCruiseVelocity(RPM.of(300));
        config.MotionMagic.withMotionMagicAcceleration(RPM.of(500).per(Seconds));

        config.Feedback
          .withSensorToMechanismRatio(kMechanismToMotorGearing);

        config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Seconds.of(0.25));
        config.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Seconds.of(0.25));

        config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(kHighSoftLimit);
        config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
        config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(kLowSoftLimit); 

        PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(config));
    }

    private void seedEncoderPosition() {
        Angle mechanismAngle = m_encoder.getAbsolutePosition().getValue().div(kMechanismToEncoderGearing);
        Angle motorAngle = mechanismAngle;
        m_motor.setPosition(motorAngle);
    }

    private boolean getIsCurrentSpiking() {
        return Amps.of(m_currentSpikeFilter.calculate(m_motor.getStatorCurrent().getValue().in(Amps))).gt(Amps.of(20));
    }

    public Angle getAngle() {
        return m_motor.getPosition().getValue();
    }

    public Command setVoltage(Voltage volts) {
        return Commands.run(() -> m_motor.setVoltage(volts.in(Volts)));
    }

    public Trigger getCurrentSpikeTrigger() {
        return m_currentSpikeTrigger;
    }

    public Trigger isNearTrigger(Supplier<Angle> angle, Angle threshHold) {
        return new Trigger(() -> getAngle().isNear(angle.get(), threshHold));
    }

    public Command setEncoderAngleCommand(Angle angle) {
        return Commands.runOnce(() -> {m_motor.setPosition(angle); m_encoder.setPosition(kMechanismToEncoderGearing);});
    }
    
    public Command zeroEncoderAngleCommand() {
        return setEncoderAngleCommand(Degrees.zero());
    }

    public Command setAngleCommand(Angle angle) {
        Angle clampedAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), kLowSoftLimit.in(Degrees), kHighSoftLimit.in(Degrees)));
        return Commands.run(() -> {
            m_motor.setControl(m_positionRequest.withPosition(clampedAngle));
        }, this);
    }

    public Command setAngleCommand(Supplier<Angle> angle) {
        return Commands.run(() -> {
            Angle clampedAngle = Degrees.of(MathUtil.clamp(angle.get().in(Degrees), kLowSoftLimit.in(Degrees), kHighSoftLimit.in(Degrees)));
            m_motor.setControl(m_positionRequest.withPosition(clampedAngle));
        }, this);
    }
}