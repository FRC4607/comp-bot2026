// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.Commands.DepotTrenchShot;
import frc.robot.Commands.HubShot;
import frc.robot.Commands.LeftMoveHoodToPosition;
import frc.robot.Commands.MoveIntakeToPosition;
import frc.robot.Commands.OutpostShot;
import frc.robot.Commands.OutpostTrenchShot;
import frc.robot.Commands.PassWithGyro;
import frc.robot.Commands.LeftRunFlywheelOpenLoop;
import frc.robot.Commands.LeftSetChamberOpenLoop;
import frc.robot.Commands.LeftSetChamberVelocity;
import frc.robot.Commands.SetIndexerOpenLoop;
import frc.robot.Commands.SetIntakeWheelsOpenLoop;
import frc.robot.Commands.SetIntakeWheelsVelocity;
import frc.robot.Commands.GeneralShot;
import frc.robot.Commands.RightMoveHoodToPosition;
import frc.robot.Commands.RightRunFlywheelOpenLoop;
import frc.robot.Commands.RightSetChamberOpenLoop;
import frc.robot.Commands.RightSetChamberVelocity;
import frc.robot.Commands.RightZeroHoodSequence;
import frc.robot.Commands.WheelRadiusCalibration;
import frc.robot.Commands.LeftZeroHoodSequence;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LeftFlywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.LeftTurret;
import frc.robot.subsystems.RightChamber;
import frc.robot.subsystems.RightFlywheel;
import frc.robot.subsystems.RightHood;
import frc.robot.subsystems.RightTurret;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    public final Joystick m_operator = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LeftFlywheel m_leftFlywheel = new LeftFlywheel();
    public final LeftHood m_leftHood = new LeftHood();
    public final IntakeArm m_intakeArm = new IntakeArm();
    public final IntakeWheels m_intakeWheels = new IntakeWheels();
    public final Indexer m_indexer = new Indexer();
    public final LeftChamber m_leftChamber = new LeftChamber();
    public final LeftTurret m_leftTurret = new LeftTurret();
    public final RightFlywheel m_rightFlywheel = new RightFlywheel();
    public final RightHood m_rightHood = new RightHood();
    public final RightChamber m_rightChamber = new RightChamber();
    public final RightTurret m_rightTurret = new RightTurret();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Trench Outpost Shot",
            new OutpostTrenchShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber));
        NamedCommands.registerCommand("Trench Depot Shot",
            new DepotTrenchShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber));
        NamedCommands.registerCommand("Outpost Shot",
            new OutpostShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber));
        NamedCommands.registerCommand("Hub Shot",
            new HubShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber));
        NamedCommands.registerCommand("Stop Shooting",
            new ParallelDeadlineGroup(
                new LeftZeroHoodSequence(m_leftHood),
                new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel),
                new SetIndexerOpenLoop(() -> 0, m_indexer),
                new LeftSetChamberOpenLoop(() -> 0, m_leftChamber),
                new RightZeroHoodSequence(m_rightHood),
                new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel),
                new RightSetChamberOpenLoop(() -> 0, m_rightChamber)).withTimeout(0.5));
        NamedCommands.registerCommand("Lower Intake Arm",
            new MoveIntakeToPosition(72, 10, m_intakeArm).withTimeout(2)
            .alongWith(new SetIntakeWheelsVelocity(5, 10, m_intakeWheels)));
        NamedCommands.registerCommand("Intake", 
            new MoveIntakeToPosition(130, 10, m_intakeArm).withTimeout(2)
            .andThen(new SetIntakeWheelsVelocity(90, 1, m_intakeWheels).withTimeout(1)));
        NamedCommands.registerCommand("Stop Intaking",
            new SetIntakeWheelsVelocity(5, 10, m_intakeWheels));
        NamedCommands.registerCommand("Raise Intake Arm",
            new MoveIntakeToPosition(0, 5, m_intakeArm).withTimeout(2));

        
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("Intake Speed", 90.0);
    }

    private void configureBindings() {

        Trigger operatorRedL = new Trigger(() -> m_operator.getRawButton(1));
        Trigger operatorRedR = new Trigger(() -> m_operator.getRawButton(2));

        Trigger operatorLKnobDown = new Trigger(() -> m_operator.getRawButton(21));
        Trigger operatorLKnobUp = new Trigger(() -> m_operator.getRawButton(22));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed).withDeadband(0.12 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed).withDeadband(0.12 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate).withDeadband(0.12 * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // reset the field-centric heading on start press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.back().onTrue(new MoveIntakeToPosition(0, 10, m_intakeArm)
            .alongWith(new SetIntakeWheelsOpenLoop(() -> 0.0, m_intakeWheels)));

        joystick.rightBumper()
            .onTrue(new MoveIntakeToPosition(130, 20, m_intakeArm)
                .alongWith(new SetIntakeWheelsVelocity(90, 80, m_intakeWheels))
                /*.alongWith(new SetIndexerOpenLoop(() -> 60.0, m_indexer)) */)
            .onFalse(new SetIntakeWheelsVelocity(10, 10, m_intakeWheels)
                .alongWith(new MoveIntakeToPosition(0, 10, m_intakeArm))
                /*.alongWith(new SetIndexerOpenLoop(() -> 0.0, m_indexer)) */);

        joystick.leftBumper().onTrue(new SetIntakeWheelsVelocity(-10, 10, m_intakeWheels))
            .onFalse(new SetIntakeWheelsVelocity(0, 10, m_intakeWheels));

        // joystick.y().onTrue(new PassWithGyro(drivetrain, m_indexer, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel))
        //     .onFalse(new RunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
        //         .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
        //         .alongWith(new SetChamberVelocity(0, 90, m_leftChamber)
        //         .alongWith(new MoveHoodToPosition(0, 0.1, m_leftHood)))));
        
        operatorRedL.onTrue(new PassWithGyro(drivetrain, m_indexer, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood)))));

        joystick.y().onTrue(new GeneralShot(drivetrain, m_indexer, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood))
                .alongWith(new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel))
                .alongWith(new RightSetChamberVelocity(0, 90, false, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
                .alongWith(new RightMoveHoodToPosition(0, 0.1, m_rightHood)))));

        // operatorRedR.onTrue(new StationaryShot(drivetrain, m_indexer, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel))
        //     .onFalse(new RunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
        //         .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
        //         .alongWith(new SetChamberVelocity(0, 90, m_leftChamber)
        //         .alongWith(new MoveHoodToPosition(0, 0.1, m_leftHood)))));

        joystick.a().onTrue(new HubShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood))
                .alongWith(new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel))
                .alongWith(new RightSetChamberVelocity(0, 90, false, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
                .alongWith(new RightMoveHoodToPosition(0, 0.1, m_rightHood)))));

        joystick.b().onTrue(new OutpostShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood))
                .alongWith(new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel))
                .alongWith(new RightSetChamberVelocity(0, 90, false, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
                .alongWith(new RightMoveHoodToPosition(0, 0.1, m_rightHood)))));

        joystick.axisGreaterThan(3, 0.8).onTrue(new OutpostTrenchShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood))
                .alongWith(new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel))
                .alongWith(new RightSetChamberVelocity(0, 90, false, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
                .alongWith(new RightMoveHoodToPosition(0, 0.1, m_rightHood)))));

        joystick.axisGreaterThan(2, 0.8).onTrue(new DepotTrenchShot(m_leftFlywheel, m_leftHood, m_leftTurret, m_indexer, m_leftChamber, m_rightFlywheel, m_rightHood, m_rightTurret, m_rightChamber))
            .onFalse(new LeftRunFlywheelOpenLoop(() -> 0, m_leftFlywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new LeftSetChamberVelocity(0, 90, false, m_leftChamber, m_leftTurret, m_leftHood, m_leftFlywheel)
                .alongWith(new LeftMoveHoodToPosition(0, 0.1, m_leftHood))
                .alongWith(new RightRunFlywheelOpenLoop(() -> 0, m_rightFlywheel))
                .alongWith(new RightSetChamberVelocity(0, 90, false, m_rightChamber, m_rightTurret, m_rightHood, m_rightFlywheel))
                .alongWith(new RightMoveHoodToPosition(0, 0.1, m_rightHood)))));

        // A command to find the radius of the wheels.
        //joystick.povRight().onTrue(new WheelRadiusCalibration(drivetrain, drive));

        joystick.back().onTrue(new LeftZeroHoodSequence(m_leftHood));

        operatorLKnobUp.onTrue(new InstantCommand(
            () -> SmartDashboard.putNumber(
                ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                SmartDashboard.getNumber(
                    ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kLeftFlywheelDistanceMult) 
                + 0.1)));
        operatorLKnobDown.onTrue(new InstantCommand(
            () -> SmartDashboard.putNumber(
                ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                SmartDashboard.getNumber(
                    ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kLeftFlywheelDistanceMult) 
                - 0.1)));

        // SmartDashboard Commands
        SmartDashboard.putData("Wheel Radius Calibration", new WheelRadiusCalibration(drivetrain, drive));
        SmartDashboard.putData("Reset Turret Position", new InstantCommand(() -> m_leftTurret.resetsetPosition()));
        SmartDashboard.putData("Reset Right Turret Position", new InstantCommand(() -> m_rightTurret.resetsetPosition()));
        SmartDashboard.putData("Zero Hood", new LeftZeroHoodSequence(m_leftHood));
        SmartDashboard.putData("Zero Right Hood", new RightZeroHoodSequence(m_rightHood));
    }

    public Command getAutonomousCommand() {
        return  autoChooser.getSelected();
    }
}
