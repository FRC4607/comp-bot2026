// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command to set the intake wheels to a specific velocity and wait until they reach the target.
 * The command finishes when the wheel velocity is within the specified tolerance.
 */
public class SetIntakeWheelsVelocity extends Command {

private double m_setpoint;
private double m_tolerance;
private IntakeWheels m_intakeWheels;

  /**
   * Creates a new SetIntakeWheelsVelocity command.
   *
   * @param setpoint the target velocity in rotations per second
   * @param tolerance the velocity tolerance for command completion
   * @param intakeWheels the intake wheels subsystem to control
   */
  public SetIntakeWheelsVelocity(double setpoint, double tolerance, IntakeWheels intakeWheels) {

    m_setpoint = setpoint;
    m_tolerance = tolerance;
    m_intakeWheels = intakeWheels;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeWheels);
  }

  @Override
  public void initialize() {
    m_intakeWheels.updateSetpoint(m_setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(m_setpoint - m_intakeWheels.getVelocity()) < m_tolerance;
  }
}
