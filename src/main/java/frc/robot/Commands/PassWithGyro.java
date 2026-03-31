// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.PassWithGyroCalibrations;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** PassWithGyro command group. */
public class PassWithGyro extends ParallelCommandGroup {
    
    /** Creates a new PassWithGyro. */
    public PassWithGyro(CommandSwerveDrivetrain drivetrain, Indexer indexer, LeftChamber leftChamber, LeftTurret leftTurret, LeftHood leftHood,
            Flywheel flywheel) {
        super(
                new LeftMoveTurretToPosition(
                        () -> drivetrain.getState().Pose.getRotation().getDegrees(),
                        0,
                        leftTurret),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new MoveHoodToPosition(
                                        PassWithGyroCalibrations.kLeftHoodAngle,
                                        PassWithGyroCalibrations.kLeftHoodTolerance,
                                        leftHood)
                                        .withTimeout(
                                                PassWithGyroCalibrations.kLeftHoodTimeout),
                                new SetFlywheelVelocity(
                                        () -> PassWithGyroCalibrations.kFlywheelVelocity,
                                        PassWithGyroCalibrations.kFlywheelVelocityTolerance,
                                        flywheel)
                                        .withTimeout(
                                                PassWithGyroCalibrations.kFlywheelTimeout)),
                        new ParallelCommandGroup(
                                new LeftSetChamberVelocity(
                                        PassWithGyroCalibrations.kLeftChamberVelocity,
                                        PassWithGyroCalibrations.kLeftChamberVelocityTolerance,
                                        true, leftChamber, leftTurret, leftHood, flywheel),
                                new SetIndexerVelocity(
                                        PassWithGyroCalibrations.kIndexerVelocity,
                                        PassWithGyroCalibrations.kIndexerVelocityTolerance,
                                        indexer))));

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
