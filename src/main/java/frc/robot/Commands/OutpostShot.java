// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostShotCalibrations;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;

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
     * @param leftHood     The leftHood to use
     * @param leftTurret   The leftTurret to use
     * @param indexer  The indexer to use
     * @param leftChamber  The leftChamber to use
     */
    public OutpostShot(Flywheel flywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber) {
        super(
            // Spin up flywheel, move leftHood, move turrets
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    () -> OutpostShotCalibrations.kFlywheelVelocity,
                    OutpostShotCalibrations.kFlywheelVelocityTolerance, 
                    flywheel).withTimeout(0.25),
                new MoveHoodToPosition(
                    OutpostShotCalibrations.kLeftHoodAngle,
                    OutpostShotCalibrations.kLeftHoodAngleTolerance, 
                    leftHood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> OutpostShotCalibrations.kLeftTurretAngle,
                    OutpostShotCalibrations.kLeftTurretAngleTolerance, 
                    leftTurret).withTimeout(0.25)),
            // Once turret, flywheel and leftHood are prepped, run indexer and chamber.
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    OutpostShotCalibrations.kIndexerVelocity, 
                    OutpostShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    OutpostShotCalibrations.kLeftChamberVelocity,
                    OutpostShotCalibrations.kLeftChamberVelocityTolerance, 
                    false, leftChamber, leftTurret, leftHood, flywheel)));
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
