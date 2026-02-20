// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveHoodToPosition extends Command {
  private double m_setpoint;
  private double m_tolerance;
  private Hood m_hood;

  // TODO: Convert to degrees once converted in the hood subsystem

  /** 
   * A command to set the closed loop setpoint of the hood, in motor rotations.
   * 
   * @param setpoint The position to drive to (motor rotations)
   * @param tolerance The tolerance for error (motor rotations)
   * @param hood The hood to use.
   */
  public MoveHoodToPosition(double setpoint, double tolerance, Hood hood) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_hood = hood;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hood.updateSetpoint(m_setpoint);
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
    return Math.abs(m_hood.getPosition()- m_setpoint) < m_tolerance;
  }
}
