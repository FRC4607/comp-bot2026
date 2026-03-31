// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.DepotTrenchShotCalibrations;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** The DepotTrenchShot sequential command group. */
public class DepotTrenchShot extends SequentialCommandGroup {
    /**
     * A command sequence to score from the depot-side trench.
     *
     * @param flywheel The flywheel to use
     * @param hood     The hood to use
     * @param leftTurret   The leftTurret to use
     * @param indexer  The indexer to use
     * @param chamber  The chamber to use
     */
    public DepotTrenchShot(Flywheel flywheel, Hood hood, LeftTurret leftTurret, Indexer indexer, Chamber chamber) {
        super(
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    () -> DepotTrenchShotCalibrations.kFlywheelVelocity,
                    DepotTrenchShotCalibrations.kFlywheelVelocityTolerance,
                    flywheel).withTimeout(0.25),
                new MoveHoodToPosition(
                    DepotTrenchShotCalibrations.kHoodAngle,
                    DepotTrenchShotCalibrations.kHoodAngleTolerance, 
                    hood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> DepotTrenchShotCalibrations.kTurretAngle,
                    DepotTrenchShotCalibrations.kTurretAngleTolerance, 
                    leftTurret).withTimeout(0.25)),
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    DepotTrenchShotCalibrations.kIndexerVelocity,
                    DepotTrenchShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new SetChamberVelocity(
                    DepotTrenchShotCalibrations.kChamberVelocity,
                    DepotTrenchShotCalibrations.kChamberVelocityTolerance,
                    false, chamber, leftTurret, hood, flywheel))
        );
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
