// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.Commands.RunFlywheelOpenLoop;
import frc.robot.Calibrations.DrivetrainCalibrations;
import frc.robot.Commands.SetHoodOpenLoop;
import frc.robot.Commands.SetIndexerOpenLoop;
import frc.robot.Commands.SetIndexerVelocity;
import frc.robot.Commands.MoveIntakeToPosition;
import frc.robot.Commands.SetIntakeWheelsOpenLoop;
import frc.robot.Commands.SetIntakeWheelsVelocity;
import frc.robot.Commands.RunTurretOpenLoop;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeManifold;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Flywheel m_flywheel = new Flywheel();
    public final Hood m_hood = new Hood();
    private final IntakeManifold m_intakeManifold = new IntakeManifold();
    private final IntakeWheels m_IntakeWheels = new IntakeWheels();
    public final Indexer m_indexer = new Indexer();
    public final Turret m_Turret = new Turret();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed).withDeadband(0.1 * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed).withDeadband(0.1 * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate).withDeadband(0.1 * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.axisGreaterThan(3, 0.1).onTrue(new RunFlywheelOpenLoop(() -> (joystick.getRightTriggerAxis()), m_flywheel));

        joystick.axisGreaterThan(2, 0.1).onTrue(new SetIntakeWheelsOpenLoop(() -> (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()), m_IntakeWheels));
        joystick.axisGreaterThan(3, 0.1).onTrue(new SetIntakeWheelsOpenLoop(() -> (joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()), m_IntakeWheels));

        joystick.axisGreaterThan(2, 0.1).onTrue(new SetIndexerOpenLoop((() -> joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()), m_indexer));
        joystick.axisGreaterThan(3, 0.1).onTrue(new SetIndexerOpenLoop((() -> joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()), m_indexer));

        joystick.back().onTrue(new MoveIntakeToPosition(0, 10, m_intakeManifold));

        joystick.x().onTrue(new MoveIntakeToPosition(80, 50, m_intakeManifold).andThen(new SetIntakeWheelsVelocity(25, 80, m_IntakeWheels)))
            .onFalse(new SetIntakeWheelsOpenLoop(() -> 0, m_IntakeWheels));

        drivetrain.registerTelemetry(logger::telemeterize);






    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
