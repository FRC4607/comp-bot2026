// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.TreeMap;

import frc.robot.Calibrations.ChinUpCalibrations;
import frc.robot.Calibrations.ClimbSequenceCalibrations;
import frc.robot.Calibrations.DrivetrainCalibrations;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.Commands.ClimbSequence;
import frc.robot.Commands.DepotTrenchShot;
import frc.robot.Commands.HubShot;
import frc.robot.Commands.MoveHoodToPosition;
import frc.robot.Commands.MoveInnerClimberToPosition;
import frc.robot.Commands.MoveIntakeToPosition;
import frc.robot.Commands.MoveOuterClimberToPosition;
import frc.robot.Commands.MoveTurretToPosition;
import frc.robot.Commands.OutpostShot;
import frc.robot.Commands.OutpostTrenchShot;
import frc.robot.Commands.PassWithGyro;
import frc.robot.Commands.RunFlywheelOpenLoop;
import frc.robot.Commands.RunTurretOpenLoop;
import frc.robot.Commands.SetHoodOpenLoop;
import frc.robot.Commands.SetIndexerOpenLoop;
import frc.robot.Commands.SetIndexerVelocity;
import frc.robot.Commands.SetInnerClimberAmperage;
import frc.robot.Commands.SetIntakeWheelsOpenLoop;
import frc.robot.Commands.SetIntakeWheelsVelocity;
import frc.robot.Commands.SetOuterClimberAmperage;
import frc.robot.Commands.StationaryShot;
import frc.robot.Commands.WheelRadiusCalibration;
import frc.robot.Commands.ZeroClimbersSequence;
import frc.robot.Commands.ZeroHood;
import frc.robot.Commands.ZeroHoodSequence;
import frc.robot.Commands.SetChamberOpenLoop;
import frc.robot.Commands.SetChamberVelocity;
import frc.robot.Commands.SetFlywheelVelocity;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Turret;

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
    public final ClimberInner m_climberInner = new ClimberInner();
    public final ClimberOuter m_climberOuter = new ClimberOuter();
    public final Flywheel m_flywheel = new Flywheel();
    public final Hood m_hood = new Hood();
    public final IntakeArm m_intakeArm = new IntakeArm();
    public final IntakeWheels m_intakeWheels = new IntakeWheels();
    public final Indexer m_indexer = new Indexer();
    public final Chamber m_chamber = new Chamber();
    public final Turret m_turret = new Turret();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Trench Outpost Shot", 
            new OutpostTrenchShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber));
        NamedCommands.registerCommand("Trench Depot Shot",
            new DepotTrenchShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber));
        NamedCommands.registerCommand("Outpost Shot", 
            new OutpostShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber));
        NamedCommands.registerCommand("Hub Shot", 
            new HubShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber));
        NamedCommands.registerCommand("Stop Shooting", 
            new ParallelDeadlineGroup(
                new ZeroHoodSequence(m_hood),
                new RunFlywheelOpenLoop(() -> 0, m_flywheel),
                new SetIndexerOpenLoop(() -> 0, m_indexer),
                new SetChamberOpenLoop(() -> 0, m_chamber)).withTimeout(0.5));
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

        // joystick.y().onTrue(new PassWithGyro(drivetrain, m_indexer, m_chamber, m_turret, m_hood, m_flywheel))
        //     .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
        //         .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
        //         .alongWith(new SetChamberVelocity(0, 90, m_chamber)
        //         .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));
        
        operatorRedL.onTrue(new PassWithGyro(drivetrain, m_indexer, m_chamber, m_turret, m_hood, m_flywheel))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        joystick.y().onTrue(new StationaryShot(drivetrain, m_indexer, m_chamber, m_turret, m_hood, m_flywheel))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        // operatorRedR.onTrue(new StationaryShot(drivetrain, m_indexer, m_chamber, m_turret, m_hood, m_flywheel))
        //     .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
        //         .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
        //         .alongWith(new SetChamberVelocity(0, 90, m_chamber)
        //         .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        joystick.a().onTrue(new HubShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        joystick.b().onTrue(new OutpostShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        joystick.axisGreaterThan(3, 0.8).onTrue(new OutpostTrenchShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        joystick.axisGreaterThan(2, 0.8).onTrue(new DepotTrenchShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel)
                .alongWith(new SetIndexerOpenLoop(() -> 0, m_indexer)
                .alongWith(new SetChamberVelocity(0, 90, false, m_chamber, m_turret, m_hood, m_flywheel)
                .alongWith(new MoveHoodToPosition(0, 0.1, m_hood)))));

        // Climb
        // joystick.povUp().onTrue(
        //     new MoveTurretToPosition(() -> 270, 1, m_turret)
        //     .alongWith(new MoveIntakeToPosition(0, 10, m_intakeArm)).withTimeout(3)
        //     .andThen(new ClimbSequence(m_climberOuter, m_climberInner)));

        // Chin up
        joystick.povRight().onTrue(
            new MoveTurretToPosition(() -> 270, 1, m_turret)
            .alongWith(new MoveIntakeToPosition(0, 10, m_intakeArm)).withTimeout(3)
            .andThen(new MoveOuterClimberToPosition(
            ChinUpCalibrations.kOuterChinUpPosition, ChinUpCalibrations.kOuterChinUpTolerance, m_climberOuter)));

        // Alignment Check
        joystick.povLeft().onTrue(
            new MoveTurretToPosition(() -> 270, 1, m_turret)
            .alongWith(new MoveIntakeToPosition(0, 10, m_intakeArm)).withTimeout(3)
            .andThen(new MoveOuterClimberToPosition(8.25, 1, m_climberOuter)));

        // Reset Climbers
        joystick.povDown().onTrue(
            new MoveTurretToPosition(() -> 270, 1, m_turret)
            .alongWith(new MoveIntakeToPosition(0, 10, m_intakeArm)).withTimeout(3)
            .andThen(new MoveInnerClimberToPosition(0.5, 10, m_climberInner)
            .alongWith(new MoveOuterClimberToPosition(1, 10, m_climberOuter))));

        //joystick.povRight().onTrue(new WheelRadiusCalibration(drivetrain, drive));

        joystick.back().onTrue(new ZeroHoodSequence(m_hood));

        operatorLKnobUp.onTrue(new InstantCommand(
            () -> Preferences.setDouble(
                ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(
                    ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kFlywheelDistanceMult) 
                + 0.1)));
        operatorLKnobDown.onTrue(new InstantCommand(
            () -> Preferences.setDouble(
                ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(
                    ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kFlywheelDistanceMult) 
                - 0.1)));

        // SmartDashboard Commands
        SmartDashboard.putData("Wheel Radius Calibration", new WheelRadiusCalibration(drivetrain, drive));
        SmartDashboard.putData("Reset Turret Position", new InstantCommand(() -> m_turret.resetsetPosition()));
        SmartDashboard.putData("Zero Hood", new ZeroHoodSequence(m_hood));

        SmartDashboard.putData("Increase Flywheel Distance Multiplier", new InstantCommand(
            () -> Preferences.setDouble(
                ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(
                    ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kFlywheelDistanceMult) 
                + 0.1)));
        
        SmartDashboard.putData("Decrease Flywheel Distance Multiplier", new InstantCommand(
            () -> Preferences.setDouble(
                ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(
                    ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kFlywheelDistanceMult) 
                - 0.1)));
    }

    public Command getAutonomousCommand() {
        return  autoChooser.getSelected();
    }
}
