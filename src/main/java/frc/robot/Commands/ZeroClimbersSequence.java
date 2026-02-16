// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroClimbersSequence extends SequentialCommandGroup {
  /** Creates a new ZeroClimbersSequence. */
  public ZeroClimbersSequence(ClimberInner climberInner, ClimberOuter climberOuter) {
    super(
      new ParallelCommandGroup(
        new SetInnerClimberAmperage(() -> 2, climberInner),
        new SetOuterClimberAmperage(() -> 2, climberOuter)
      ).withTimeout(1),
      new ZeroClimbers(climberInner, climberOuter)
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
