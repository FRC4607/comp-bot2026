// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chamber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetChamberVelocity extends Command {
  private double m_setpoint;
  private double m_tolerance;
  private Chamber m_chamber;

  /** Creates a new SetChamberVelocity. */
  public SetChamberVelocity(double setpoint, double tolerance, Chamber chamber) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_chamber = chamber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chamber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chamber.updateSetpoint(m_setpoint);
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
    return Math.abs(m_chamber.getVelocity() - m_setpoint) < m_tolerance;
  }
}
