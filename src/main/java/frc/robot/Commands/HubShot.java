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
public class HubShot extends SequentialCommandGroup {
  
  /** 
   * A Command sequence that will shoot into the Hub when the robot is touching the hub with the climbers directly opposite of the hub wall.
   * 
   * @param flywheel The flywheel to use
   * @param hood The hood to use
   * @param turret The turret to use
   * @param indexer The indexer to use
   * @param chamber The chamber to use
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
