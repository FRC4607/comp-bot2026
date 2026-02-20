// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.ToLongBiFunction;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to set the flywheel to a specific velocity and wait until it reaches the target.
 * The command finishes when the flywheel velocity is within the specified tolerance.
 */
public class SetFlywheelVelocity extends Command {
  private double m_setpoint;
  private double m_tolerance;
  private Flywheel m_flywheel;

  /**
   * Creates a new SetFlywheelVelocity command.
   *
   * @param setpoint the target velocity in rotations per second
   * @param tolerance the velocity tolerance for command completion
   * @param flywheel the flywheel subsystem to control
   */
  public SetFlywheelVelocity(double setpoint, double tolerance, Flywheel flywheel) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  @Override
  public void initialize() {
    m_flywheel.updateSetpoint(m_setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_flywheel.getVelocity() - m_setpoint) < m_tolerance;
  }
}
