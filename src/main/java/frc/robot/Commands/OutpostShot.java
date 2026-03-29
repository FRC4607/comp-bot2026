// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostShotCalibrations;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** OutpostShot sequential command group. */
public class OutpostShot extends SequentialCommandGroup {

    /**
     * A Command sequence that will score when the robot is touching the outpost
     * corner with the intake facing the tower.
     *
     * @param flywheel The flywheel to use
     * @param hood     The hood to use
     * @param turret   The turret to use
     * @param indexer  The indexer to use
     * @param chamber  The chamber to use
     */
    public OutpostShot(Flywheel flywheel, Hood hood, Turret turret, Indexer indexer, Chamber chamber) {
        super(
            // Spin up flywheel, move hood, move turret
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    () -> OutpostShotCalibrations.kFlywheelVelocity,
                    OutpostShotCalibrations.kFlywheelVelocityTolerance, 
                    flywheel).withTimeout(0.25),
                new MoveHoodToPosition(
                    OutpostShotCalibrations.kHoodAngle,
                    OutpostShotCalibrations.kHoodAngleTolerance, 
                    hood).withTimeout(0.25),
                new MoveTurretToPosition(
                    () -> OutpostShotCalibrations.kTurretAngle,
                    OutpostShotCalibrations.kTurretAngleTolerance, 
                    turret).withTimeout(0.25)),
            // Once turret, flywheel and hood are prepped, run indexer and chamber.
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    OutpostShotCalibrations.kIndexerVelocity, 
                    OutpostShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new SetChamberVelocity(
                    OutpostShotCalibrations.kChamberVelocity,
                    OutpostShotCalibrations.kChamberVelocityTolerance, 
                    false, chamber, turret, hood, flywheel)));
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
