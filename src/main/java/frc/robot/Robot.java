// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class responsible for the overall robot lifecycle and timing.
 * This class extends {@link TimedRobot} and runs at 50Hz, calling the appropriate
 * methods for each robot state (disabled, autonomous, teleop, test).
 */
public class Robot extends TimedRobot {
  /** The autonomous command to be scheduled */
  private Command m_autonomousCommand;

  /** The robot container with all subsystems and commands */
  public final RobotContainer m_robotContainer;

  /**
   * Constructs the Robot and initializes the RobotContainer.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode.
   * Use this to update universal telemetry, process inputs for diagnostics, etc.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes. This will be called each time the robot enters autonomous mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.m_turret.resetsetPosition();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** Called when the disabled period ends. */
  @Override
  public void disabledExit() {}

  /**
   * This autonomous routine is run when the robot is in autonomous mode.
   * The selected autonomous command will be scheduled at this time.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when the autonomous period ends. */
  @Override
  public void autonomousExit() {}

  /**
   * This function is called at the beginning of operator control.
   * If an autonomous command is running, it will be cancelled.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the teleop period ends. */
  @Override
  public void teleopExit() {}

  /**
   * This function is called at the beginning of test mode.
   * All commands are cancelled when test mode is entered.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the test period ends. */
  @Override
  public void testExit() {}

  /** This function is called periodically during simulation. */
  @Override
  public void simulationPeriodic() {}
}
