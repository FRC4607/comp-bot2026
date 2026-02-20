// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to run the indexer in open-loop mode with a variable duty cycle from a supplier.
 * This command does not finish on its own; it must be interrupted or cancelled.
 */
public class SetIndexerOpenLoop extends Command {
  private DoubleSupplier m_dutyCycle;
  private Indexer m_indexer;
  
  /**
   * Creates a new SetIndexerOpenLoop command.
   *
   * @param dutyCycle a supplier providing the duty cycle value from -1.0 to 1.0
   * @param indexer the indexer subsystem to control
   */
  public SetIndexerOpenLoop(DoubleSupplier dutyCycle, Indexer indexer) {
    m_dutyCycle = dutyCycle;
    m_indexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_indexer.runOpenLoop(m_dutyCycle.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
