// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberOuter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetOuterClimberAmperage extends Command {

  private DoubleSupplier m_amperage;
  private ClimberOuter m_climberOuter;

  /** Creates a new SetOuterClimberAmperage. */
  public SetOuterClimberAmperage(DoubleSupplier amperage, ClimberOuter climberOuter) {
    m_amperage = amperage;
    m_climberOuter = climberOuter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberOuter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberOuter.runOpenLoop(m_amperage.getAsDouble());
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
