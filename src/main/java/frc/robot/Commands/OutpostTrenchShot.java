// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostTrenchShotCalibrations;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** OutpostTrenchShot sequential command group. */
public class OutpostTrenchShot extends SequentialCommandGroup {
    /**
     * A command sequence to score from the outpost-side trench.
     *
     * @param flywheel The flywheel to use
     * @param leftHood     The leftHood to use
     * @param leftTurret   The leftTurret to use
     * @param indexer  The indexer to use
     * @param leftChamber  The leftChamber to use
     */
    public OutpostTrenchShot(Flywheel flywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber) {
        super(
            // Spin up flywheel, move leftHood, move turret
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    () -> OutpostTrenchShotCalibrations.kFlywheelVelocity,
                    OutpostTrenchShotCalibrations.kFlywheelVelocityTolerance, 
                    flywheel).withTimeout(0.25),
                new MoveHoodToPosition(
                    OutpostTrenchShotCalibrations.kLeftHoodAngle,
                    OutpostTrenchShotCalibrations.kLeftHoodAngleTolerance, 
                    leftHood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> OutpostTrenchShotCalibrations.kLeftTurretAngle,
                    OutpostTrenchShotCalibrations.kLeftTurretAngleTolerance, 
                    leftTurret).withTimeout(0.25)),
            // Once turret, flywheel and leftHood are prepped, run indexer and chamber.
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    OutpostTrenchShotCalibrations.kIndexerVelocity, 
                    OutpostTrenchShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    OutpostTrenchShotCalibrations.kLeftChamberVelocity,
                    OutpostTrenchShotCalibrations.kLeftChamberVelocityTolerance, 
                    false, leftChamber, leftTurret, leftHood, flywheel)));
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
    
}


