// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveTurretToPosition command. */
public class MoveTurretToPosition extends Command {
    private DoubleSupplier m_degrees;
    private double m_tolerance;
    private Turret m_turret;

    /**
     * A command that sets the setpoint of the turret, in degrees.
     *
     * @param degrees  The position to drive towards, in degrees.
     * @param tolerance The tolerance for error, in degrees. If set to 0, the command will not end.
     * @param turret    The turret to use
     */
    public MoveTurretToPosition(DoubleSupplier degrees, double tolerance, Turret turret) {
        m_degrees = degrees;
        m_tolerance = tolerance;
        m_turret = turret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turret.updateSetpoint(m_degrees.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Update the setpoint if the target has changed by more than 0.01 degrees
        if (Math.abs(m_degrees.getAsDouble() - m_turret.getSetpoint()) > 0.01) {
            m_turret.updateSetpoint(m_degrees.getAsDouble());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_tolerance == 0) {
            return false;
        } else {
            return Math.abs(m_turret.getPosition() - m_degrees.getAsDouble()) < m_tolerance;
        }
    }
}
