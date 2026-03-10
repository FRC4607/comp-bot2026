// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberOuter;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetOuterClimberAmperage command. */
public class SetOuterClimberAmperage extends Command {

    private DoubleSupplier m_amperage;
    private ClimberOuter m_climberOuter;

    /** 
     * A command to set the open loop amperage of the outer climber.
     *
     * @param amperage The amps to run at (-80, 80)
     * @param climberOuter The climberOuter to use.
     */
    public SetOuterClimberAmperage(DoubleSupplier amperage, ClimberOuter climberOuter) {
        m_amperage = amperage;
        m_climberOuter = climberOuter;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_climberOuter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climberOuter.runOpenLoop(m_amperage.getAsDouble());
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
