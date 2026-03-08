// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ClimbSequenceCalibrations;
import frc.robot.Calibrations.ClimberCalibrations;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** The ClimbSequence sequential command group. */
public class ClimbSequence extends SequentialCommandGroup {

    /**
     * A command sequence to climb the tower.
     *
     * @param climberOuter The outer climber
     * @param climberInner The inner climber
     */
    public ClimbSequence(ClimberOuter climberOuter, ClimberInner climberInner) {
        super(
            // l1 pull-up
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPosition,
                ClimbSequenceCalibrations.kOuterPositionTolerance, climberOuter).withTimeout(3),

            // l1 handoff
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerHandoffPosition,
                ClimbSequenceCalibrations.kInnerHandoffPositionTolerance, climberInner).withTimeout(3),

            // l2 prep
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPrep,
                ClimbSequenceCalibrations.kOuterPrepTolerance, climberOuter).withTimeout(3),
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerTraversalPosition,
                ClimbSequenceCalibrations.kInnerTraversalPositionTolerance, climberInner).withTimeout(3),

            // l2 chin-up
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterTraversalPosition,
                ClimbSequenceCalibrations.kOuterTraversalPositionTolerance, climberOuter).withTimeout(3),

            // Remove middle hook from bar
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerPrep, 
                ClimbSequenceCalibrations.kInnerPrepTolerance, climberInner).withTimeout(3),

            // l2 pull-up
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPosition, 
                ClimbSequenceCalibrations.kOuterPositionTolerance, climberOuter).withTimeout(3),

            // l2 handoff
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerHandoffPosition,
                ClimbSequenceCalibrations.kInnerHandoffPositionTolerance, climberInner).withTimeout(3),

            // l3 prep
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPrep,
                ClimbSequenceCalibrations.kOuterPrepTolerance, climberOuter).withTimeout(3),
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerTraversalPosition,
                ClimbSequenceCalibrations.kInnerTraversalPositionTolerance, climberInner).withTimeout(3),

            // l3 chin-up
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterTraversalPosition,
                ClimbSequenceCalibrations.kOuterTraversalPositionTolerance, climberOuter).withTimeout(3),

            // Remove middle hook from bar
            new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerPrep, 
                ClimbSequenceCalibrations.kInnerPrepTolerance, climberInner).withTimeout(3),

            // l3 pull-up
            new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPosition, 
                ClimbSequenceCalibrations.kOuterPositionTolerance, climberOuter).withTimeout(3)
        );
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
