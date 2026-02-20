// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to run the flywheel at a variable duty cycle from a supplier.
 * This command does not finish on its own; it must be interrupted or cancelled.
 */
public class RunFlywheelOpenLoop extends Command {

  private DoubleSupplier m_dutyCycle;
  private Flywheel m_flywheel;

  /**
   * Creates a new RunFlywheelOpenLoop command.
   *
   * @param dutyCycle a supplier providing the duty cycle value from -1.0 to 1.0
   * @param flywheel the flywheel subsystem to control
   */
  public RunFlywheelOpenLoop(DoubleSupplier dutyCycle, Flywheel flywheel) {
    m_dutyCycle = dutyCycle;
    m_flywheel = flywheel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheel);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_flywheel.runOpenLoop(m_dutyCycle.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
