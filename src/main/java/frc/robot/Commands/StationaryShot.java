// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StationaryShot extends ParallelCommandGroup {
    /** Creates a new StationaryShot. */
    public StationaryShot(
        DoubleSupplier vY, DoubleSupplier vX, CommandSwerveDrivetrain drivetrain, Indexer indexer, Chamber chamber, Turret turret, Hood hood, Flywheel flywheel) {

        super(
            new PointAtHub(vY, vX, drivetrain, turret, hood, flywheel),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                    new SetChamberVelocity(90, 3, chamber),
                    new SetIndexerVelocity(90, 3, indexer)
                )
            )
        );
    }
}
