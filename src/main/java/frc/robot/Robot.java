// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drivetrain.Drive;

public class Robot extends LoggedRobot {
  private final AutoChooser m_autoChooser;
  private final Drive m_drive;

  public Robot() {
    m_drive = new Drive(null, null, null, null, null);

    Autos autos = new Autos(m_drive);
    m_autoChooser = autos.createAutoChooser();
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {

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

  @Override
  public void simulationInit() {
  }

  @Override
  public void driverStationConnected() {
    Threads.setCurrentThreadPriority(true, 10);
  }
}
