// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** RunTurretOpenLoop command. */
public class RunTurretOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private Turret m_turret;

    /**
     * A command that runs the turret in open loop control.
     *
     * @param dutyCycle The power to run at (-1, 1)
     * @param turret    The turret to use
     */
    public RunTurretOpenLoop(DoubleSupplier dutyCycle, Turret turret) {
        m_dutyCycle = dutyCycle;
        m_turret = turret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_turret.runOpenLoop(m_dutyCycle.getAsDouble());
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
