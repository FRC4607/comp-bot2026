// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberInner;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetInnerClimberAmperage command. */
public class SetInnerClimberAmperage extends Command {

    private DoubleSupplier m_amperage;
    private ClimberInner m_climberInner;

    /** 
     * A command to set the open loop amperage of the inner climber.
     *
     * @param amperage The amperage to run at (-80, 80)
     * @param climberInner The climberInner to use
     */
    public SetInnerClimberAmperage(DoubleSupplier amperage, ClimberInner climberInner) {
        m_amperage = amperage;
        m_climberInner = climberInner;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climberInner);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climberInner.runOpenLoop(m_amperage.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
