// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to set the indexer to a specific velocity and wait until it reaches the target.
 * The command finishes when the indexer velocity is within the specified tolerance.
 */
public class SetIndexerVelocity extends Command {
  private double m_velocity;
  private double m_tolerance;
  private Indexer m_indexer;

  /**
   * Creates a new SetIndexerVelocity command.
   *
   * @param velocity the target velocity in rotations per second
   * @param tolerance the velocity tolerance for command completion
   * @param indexer the indexer subsystem to control
   */
  public SetIndexerVelocity(double velocity, double tolerance, Indexer indexer) {
    m_velocity = velocity;
    m_tolerance = tolerance;
    m_indexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer);

  }

  @Override
  public void initialize() {
    m_indexer.updateSetpoint(m_velocity);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_indexer.getVelocity()-m_velocity) < m_tolerance;

  }
}
