// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to run the hood in open-loop mode with a variable duty cycle from a supplier.
 * This command does not finish on its own; it must be interrupted or cancelled.
 */
public class SetHoodOpenLoop extends Command {
  private DoubleSupplier m_dutyCycle;
  private Hood m_hood;

  /**
   * Creates a new SetHoodOpenLoop command.
   *
   * @param dutyCycle a supplier providing the duty cycle value from -1.0 to 1.0
   * @param hood the hood subsystem to control
   */
  public SetHoodOpenLoop(DoubleSupplier dutyCycle, Hood hood) {
    m_dutyCycle = dutyCycle;
    m_hood = hood;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hood.runOpenLoop(m_dutyCycle.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
