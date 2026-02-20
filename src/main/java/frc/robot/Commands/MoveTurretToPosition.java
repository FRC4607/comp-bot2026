// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to move the turret to a specific angle and wait until it reaches the target.
 * The command finishes when the turret position is within the specified tolerance.
 */
public class MoveTurretToPosition extends Command {
  private double m_setpoint;
  private double m_tolerance;
  private Turret m_turret;

  /**
   * Creates a new MoveTurretToPosition command.
   *
   * @param setpoint the target turret angle in rotations
   * @param tolerance the position tolerance for command completion
   * @param turret the turret subsystem to control
   */
  public MoveTurretToPosition(double setpoint, double tolerance, Turret turret) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {
    m_turret.updateSetpoint(m_setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_turret.getPosition() - m_setpoint) < m_tolerance;
  }
}
