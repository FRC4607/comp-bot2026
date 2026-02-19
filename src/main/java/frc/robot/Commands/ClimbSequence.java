// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ClimbSequenceCalibrations;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public ClimbSequence(ClimberOuter climberOuter, ClimberInner climberInner) {
    super(
      // Prep Climbers
      new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPrep, ClimbSequenceCalibrations.kOuterPrepTolerance, climberOuter),
      new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerPrep, ClimbSequenceCalibrations.kInnerPrepTolerance, climberInner),

      // l1 pull-up
      new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPosition, ClimbSequenceCalibrations.kOuterPositionTolerance, climberOuter),

      // l1 handoff
      new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerHandoffPosition, ClimbSequenceCalibrations.kInnerHandoffPositionTolerance, climberInner),

      // l2 prep
      new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterPrep, ClimbSequenceCalibrations.kOuterPrepTolerance, climberOuter),
      new MoveInnerClimberToPosition(ClimbSequenceCalibrations.kInnerTraversalPosition, ClimbSequenceCalibrations.kInnerTraversalPositionTolerance, climberInner),

      // l2 chin-up
      new MoveOuterClimberToPosition(ClimbSequenceCalibrations.kOuterTraversalPosition, ClimbSequenceCalibrations.kOuterTraversalPositionTolerance, climberOuter)
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
