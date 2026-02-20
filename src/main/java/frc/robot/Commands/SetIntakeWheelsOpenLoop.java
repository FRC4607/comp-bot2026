// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to run the intake wheels in open-loop amperage mode with a variable output from a supplier.
 * This command does not finish on its own; it must be interrupted or cancelled.
 */
public class SetIntakeWheelsOpenLoop extends Command {

  private DoubleSupplier m_amperage;
  private IntakeWheels m_intakeWheels;

  /**
   * Creates a new SetIntakeWheelsOpenLoop command.
   *
   * @param amperage a supplier providing the amperage value
   * @param intakeWheels the intake wheels subsystem to control
   */
  public SetIntakeWheelsOpenLoop(DoubleSupplier amperage, IntakeWheels intakeWheels) {
    m_amperage = amperage;
    m_intakeWheels = intakeWheels;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeWheels);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intakeWheels.setOpenLoop(m_amperage.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
