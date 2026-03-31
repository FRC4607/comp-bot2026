// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LeftTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetChamberVelocity command. */
public class SetChamberVelocity extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private boolean m_waitForTurret;

    private Chamber m_chamber;
    private LeftTurret m_leftTurret;
    private Hood m_hood;
    private Flywheel m_flywheel;


    /** Creates a new SetChamberVelocity. */
    public SetChamberVelocity(double setpoint, double tolerance, boolean waitForTurret, Chamber chamber, LeftTurret leftTurret, Hood hood, Flywheel flywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_chamber = chamber;
        m_leftTurret = leftTurret;
        m_hood = hood;
        m_flywheel = flywheel;

        m_waitForTurret = waitForTurret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_chamber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_waitForTurret) {
            m_chamber.updateSetpoint(m_setpoint);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_waitForTurret) {
            if ((Math.abs(m_leftTurret.getPosition() - m_leftTurret.getSetpoint()) > 3)
                && (Math.abs(m_hood.getPosition() - m_hood.getSetpoint()) > 0.2)
                && (Math.abs(m_flywheel.getVelocity() - m_flywheel.getSetpoint()) > 3)) {
                m_chamber.updateSetpoint(0);
            } else if ((Math.abs(m_chamber.getSetpoint() - m_setpoint) > 0.1)) {
                m_chamber.updateSetpoint(m_setpoint);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_chamber.getVelocity() - m_setpoint) < m_tolerance;
    }
}
