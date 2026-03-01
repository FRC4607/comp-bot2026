// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberInner;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveInnerClimberToPosition Command. */
public class MoveInnerClimberToPosition extends Command {

    private double m_inches;
    private double m_tolerance;
    private ClimberInner m_climberInner;

    /**
     * A command to set the setpoint of the inner climber, in inches.
     *
     * @param inches     Position to drive towards (inches)
     * @param tolerance    Tolerance for error (inches)
     * @param climberInner The climberInner to use.
     */
    public MoveInnerClimberToPosition(double inches, double tolerance, ClimberInner climberInner) {
        m_inches = inches;
        m_tolerance = tolerance;
        m_climberInner = climberInner;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climberInner);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climberInner.updateSetpoint(m_inches);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_inches - m_climberInner.getPosition()) < m_tolerance;
    }
}
