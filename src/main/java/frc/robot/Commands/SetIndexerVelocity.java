// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIndexerVelocity extends Command {
  private DoubleSupplier m_velocity;
  private double m_tolerance;
  private Indexer m_indexer;

  /** Creates a new SetIndexerVelocity. */
  public SetIndexerVelocity(DoubleSupplier velocity, double tolerance, Indexer indexer) {
    m_velocity = velocity;
    m_tolerance = tolerance;
    m_indexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.updateSetpoint(m_velocity.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_indexer.getVelocity()-m_velocity.getAsDouble()) < m_tolerance;

  }
}
