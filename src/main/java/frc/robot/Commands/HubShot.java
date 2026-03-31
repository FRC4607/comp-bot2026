// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.HubShotCalibrations;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** HubShot sequential command group. */
public class HubShot extends SequentialCommandGroup {

    /**
     * A Command sequence that will shoot into the Hub when the robot is
     * touching the hub with the intake to the right
     *
     * @param flywheel The flywheel to use
     * @param hood     The hood to use
     * @param leftTurret   The leftTurret to use
     * @param indexer  The indexer to use
     * @param leftChamber  The leftChamber to use
     */
    public HubShot(Flywheel flywheel, Hood hood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber) {
        super(
            new ParallelCommandGroup(
                new SetFlywheelVelocity(
                    () -> HubShotCalibrations.kFlywheelVelocity,
                    HubShotCalibrations.kFlywheelVelocityTolerance, 
                    flywheel),
                new MoveHoodToPosition(
                    HubShotCalibrations.kHoodAngle, 
                    HubShotCalibrations.kHoodAngleTolerance,
                    hood),
                new LeftMoveTurretToPosition(
                    () -> HubShotCalibrations.kLeftTurretAngle,
                    HubShotCalibrations.kLeftTurretAngleTolerance, 
                    leftTurret)),
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    HubShotCalibrations.kIndexerVelocity, 
                    HubShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    HubShotCalibrations.kLeftChamberVelocity,
                    HubShotCalibrations.kLeftChamberVelocityTolerance, 
                    false, leftChamber, leftTurret, hood, flywheel))
        );
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
