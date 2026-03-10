// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** ZeroClimbers command. */
public class ZeroClimbers extends Command {
    private ClimberInner m_climberInner;
    private ClimberOuter m_climberOuter;
    private boolean m_isInnerAtVelocity;
    private boolean m_isOuterAtVelocity;

    /** 
     * A command to rezero the climbers when the climbers are away from the hard stop.
     *
     * @param climberInner The climberInner to use
     * @param climberOuter The climberOuter to use
     */
    public ZeroClimbers(ClimberInner climberInner, ClimberOuter climberOuter) {
        m_climberInner = climberInner;
        m_climberOuter = climberOuter;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climberInner, m_climberOuter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        m_isInnerAtVelocity = false;
        m_isOuterAtVelocity = false;

        // Drive the climbers towards the hard stop..
        m_climberInner.runOpenLoop(-2);
        m_climberOuter.runOpenLoop(-2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Make sure the climbers start moving before zeroing.
        if ((m_climberInner.getVelocity() < -1) && !m_isInnerAtVelocity) {
            m_isInnerAtVelocity = true;
        }

        if ((m_climberOuter.getVelocity() < -1) && !m_isOuterAtVelocity) {
            m_isOuterAtVelocity = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Stop the climbers.
        m_climberInner.runOpenLoop(0);
        m_climberOuter.runOpenLoop(0);

        // If this command ran to completion, zero the climbers.
        if (!interrupted) {
            m_climberInner.setPosition(0);
            m_climberOuter.setPosition(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // returns true if climbers have moved and then stopped.
        if (m_isInnerAtVelocity && m_isOuterAtVelocity && (m_climberInner.getVelocity() > -1)
                && (m_climberOuter.getVelocity() > -1)) {
            return true;
        } else {
            return false;
        }
    }
}
