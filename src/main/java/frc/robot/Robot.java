package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Controls;
import frc.robot.controls.XboxControls;
import frc.robot.generated.TunerConstants;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeObstacleProtection;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Turret;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Controls m_controls;

  private final CommandSwerveDrivetrain m_drive;
  private final Vision m_vision;

  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;

  private final Turret m_turret;

  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

  private final Feeder m_feeder;

  private final RobotState m_state;

  private final RobotViz m_viz;

  private final Alert m_obstacleProtectionAlert = new Alert("Obstacle protection failed", AlertType.kError);

  public Robot() {
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());
    DataLogManager.start();
    Epilogue.bind(this);

    m_controls = new XboxControls(0);
    m_drive = TunerConstants.createDrivetrain();
    m_feeder = new Feeder();
    m_intakePivot = new IntakePivot();
    m_intakeRoller = new IntakeRoller();
    m_turret = new Turret(0, 0);

    Camera rearFacingRightCamera = new Camera("rearFacingRightCamera", new Transform3d());
    Camera frontFacingRightCamera = new Camera("frontFacingRightCamera", new Transform3d());
    Camera rearFacingLeftCamera = new Camera("rearFacingLeftCamera", new Transform3d());
    Camera frontFacingLeftCamera = new Camera("frontFacingLeftCamera", new Transform3d());

    m_vision = new Vision(
        (observation) -> m_drive.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp()),
        m_drive::getPose,
        rearFacingRightCamera,
        frontFacingRightCamera,
        rearFacingLeftCamera,
        frontFacingLeftCamera);

    m_state = new RobotState(m_controls, m_drive, m_intakePivot, m_turret);

    m_viz = new RobotViz(m_state);

    m_autoFactory = new AutoFactory(
        m_drive::getPose, // A function that returns the current robot pose
        m_drive::resetPose, // A function that resets the current robot pose to the provided Pose2d
        m_drive::followPath, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
        m_drive // The drive subsystem
    );

    m_autoRoutines = new AutoRoutines(m_autoFactory);
    m_autoChooser = new AutoChooser("Do Nothing");
    generateAutoChooser();

    if (Constants.kEnableObstacleProtection) {
      try {
        IntakeObstacleProtection.getTrigger(m_state)
            .onTrue(m_intakePivot.setAngleCommand(Degrees.of(60)));
      } catch (IOException e) {
        m_obstacleProtectionAlert.set(true);
      }
    }

  }

  private void generateAutoChooser() {
    m_autoChooser.addRoutine("Example Movement", m_autoRoutines::exampleMovementAuto);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    new Trigger(DriverStation::isAutonomousEnabled).whileTrue(m_autoChooser.selectedCommandScheduler());

    // RobotModeTriggers.autonomous().whileTrue(m_autoRoutines.taxiAuto().cmd());
    m_controls.intakeFuel().whileTrue(runIntakeCommand()).onFalse(stowIntakeCommand());
  }

  private Command setIntakeGoalCommand(IntakeGoal goal) {
    return Commands.parallel(
        m_intakePivot.setAngleCommand(goal.angle),
        m_intakeRoller.setVoltageCommand(goal.voltage));
  }

  private Command runIntakeCommand() {
    return setIntakeGoalCommand(IntakeGoal.INTAKING);
  }

  private Command stowIntakeCommand() {
    return setIntakeGoalCommand(IntakeGoal.STOW);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_viz.periodic();

    try {
      DogLog.log("John", IntakeObstacleProtection.getTrigger(m_state).getAsBoolean());
    } catch (IOException e) {
      e.printStackTrace();
    }

    DogLog.log("Autonomous", DriverStation.isAutonomousEnabled());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
