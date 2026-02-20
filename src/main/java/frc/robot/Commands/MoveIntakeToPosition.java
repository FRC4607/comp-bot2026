// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeManifold;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to move the intake manifold to a specific position and wait until it reaches the target.
 * The command finishes when the intake position is within the specified tolerance.
 */
public class MoveIntakeToPosition extends Command {

  private final double m_setpoint;
  private final double m_tolerance;
  private final IntakeManifold m_intakeManifold;

  /**
   * Creates a new MoveIntakeToPosition command.
   *
   * @param setpoint the target intake angle in degrees
   * @param tolerance the position tolerance for command completion
   * @param intakeManifold the intake manifold subsystem to control
   */
  public MoveIntakeToPosition(double setpoint, double tolerance, IntakeManifold intakeManifold) {
    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_intakeManifold = intakeManifold;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeManifold);
  }

  @Override
  public void initialize() {
    m_intakeManifold.updateSetpoint(m_setpoint);
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
    return Math.abs(m_setpoint - m_intakeManifold.getPosition()) < m_tolerance;
  }
}
