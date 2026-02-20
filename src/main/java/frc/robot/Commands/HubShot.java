// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.HubShotCalibrations;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * HubShot command group executes a complete hub shot sequence.
 * First adjusts all subsystems in parallel (flywheel, hood, turret),
 * then fires the shot by spinning the indexer and chamber in parallel.
 */
public class HubShot extends SequentialCommandGroup {
  /**
   * Creates a new HubShot command that orchestrates a complete shot to the hub.
   *
   * @param flywheel the flywheel subsystem
   * @param hood the hood subsystem
   * @param turret the turret subsystem
   * @param indexer the indexer subsystem
   * @param chamber the chamber subsystem
   */
  public HubShot(Flywheel flywheel, Hood hood, Turret turret, Indexer indexer, Chamber chamber) {
    super(
      new ParallelCommandGroup(
        new SetFlywheelVelocity(HubShotCalibrations.kFlywheelVelocity, HubShotCalibrations.kFlywheelVelocityTolerance, flywheel),
        new MoveHoodToPosition(HubShotCalibrations.kHoodAngle, HubShotCalibrations.kHoodAngleTolerance, hood),
        new MoveTurretToPosition(HubShotCalibrations.kTurretAngle, HubShotCalibrations.kTurretAngleTolerance, turret)
        ),
      new ParallelCommandGroup(
        new SetIndexerVelocity(HubShotCalibrations.kIndexerVelocity, HubShotCalibrations.kIndexerVelocityTolerance, indexer),
        new SetChamberVelocity(HubShotCalibrations.kChamberVelocity, HubShotCalibrations.kChamberVelocityTolerance, chamber)
      )
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
