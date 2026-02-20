// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberOuter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to move the outer climber to a specific position and wait until it reaches the target.
 * The command finishes when the climber position is within the specified tolerance.
 */
public class MoveOuterClimberToPosition extends Command {
  private double m_setpoint;
  private double m_tolerance;
  private ClimberOuter m_climberOuter;
  /**
   * Creates a new MoveOuterClimber command.
   *
   * @param setpoint the target position in rotations
   * @param tolerance the position tolerance for command completion
   * @param climberOuter the outer climber subsystem to control
   */
  public MoveOuterClimberToPosition(double setpoint, double tolerance, ClimberOuter climberOuter) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_climberOuter = climberOuter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberOuter);
  }

  @Override
  public void initialize() {
    m_climberOuter.updateSetpoint(m_setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_setpoint - m_climberOuter.getPosition()) < m_tolerance;
  }
}
