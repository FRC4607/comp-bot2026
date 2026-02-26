// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostTrenchShotCalibrations;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** OutpostTrenchShot sequential command group. */
public class OutpostTrenchShot extends SequentialCommandGroup {
    /**
     * A command sequence to score from the outpost-side trench.
     *
     * @param flywheel The flywheel to use
     * @param hood     The hood to use
     * @param turret   The turret to use
     * @param indexer  The indexer to use
     * @param chamber  The chamber to use
     */
    public OutpostTrenchShot(Flywheel flywheel, Hood hood, Turret turret, Indexer indexer, Chamber chamber) {
        super(
            // Spin up flywheel, move hood, move turret
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    OutpostTrenchShotCalibrations.kFlywheelVelocity,
                    OutpostTrenchShotCalibrations.kFlywheelVelocityTolerance, 
                    flywheel),
                new MoveHoodToPosition(
                    OutpostTrenchShotCalibrations.kHoodAngle,
                    OutpostTrenchShotCalibrations.kHoodAngleTolerance, 
                    hood),
                new MoveTurretToPosition(
                    OutpostTrenchShotCalibrations.kTurretAngle,
                    OutpostTrenchShotCalibrations.kTurretAngleTolerance, 
                    turret)),
            // Once turret, flywheel and hood are prepped, run indexer and chamber.
            new ParallelCommandGroup(
                new SetIndexerOpenLoop(
                    () -> OutpostTrenchShotCalibrations.kIndexerVelocity, 
                    indexer),
                new SetChamberVelocity(
                    OutpostTrenchShotCalibrations.kChamberVelocity,
                    OutpostTrenchShotCalibrations.kChamberVelocityTolerance, 
                    chamber)));
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
