// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIntakeWheelsOpenLoop extends Command {

  private DoubleSupplier m_amperage;
  private IntakeWheels m_intakeWheels;

  /** Creates a new SetIntakeWheelsOpenLoop. */
  public SetIntakeWheelsOpenLoop(DoubleSupplier amperage, IntakeWheels intakeWheels) {
    m_amperage = amperage;
    m_intakeWheels = intakeWheels;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeWheels.setOpenLoop(m_amperage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
