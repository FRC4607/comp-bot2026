// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetFlywheelVelocity command. */
public class SetFlywheelVelocity extends Command {
    private DoubleSupplier m_setpoint;
    private double m_tolerance;
    private Flywheel m_flywheel;

    /**
     * A command to set the closed loop setpoint of the flywheel, in rotations per second.
     *
     * @param setpoint  The desired speed of the flywheel (rot/s)
     * @param tolerance The tolerance for error (rot/s)
     * @param flywheel  The flywheel to use
     */
    public SetFlywheelVelocity(DoubleSupplier setpoint, double tolerance, Flywheel flywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_flywheel = flywheel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_flywheel.updateSetpoint(m_setpoint.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(m_setpoint.getAsDouble() - m_flywheel.getSetpoint()) > 0.01) {
            m_flywheel.updateSetpoint(m_setpoint.getAsDouble());
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
            return Math.abs(m_flywheel.getVelocity() - m_setpoint.getAsDouble()) < m_tolerance;
        }
    }
}
