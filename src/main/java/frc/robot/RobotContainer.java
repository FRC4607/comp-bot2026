// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Calibrations.DrivetrainCalibrations;
import frc.robot.Commands.ClimbSequence;
import frc.robot.Commands.HubShot;
import frc.robot.Commands.MoveHoodToPosition;
import frc.robot.Commands.MoveInnerClimberToPosition;
import frc.robot.Commands.MoveOuterClimberToPosition;
import frc.robot.Commands.MoveTurretToPosition;
import frc.robot.Commands.RunFlywheelOpenLoop;
import frc.robot.Commands.SetInnerClimberAmperage;
import frc.robot.Commands.SetOuterClimberAmperage;
import frc.robot.Commands.WheelRadiusCalibration;
import frc.robot.Commands.ZeroClimbersSequence;
import frc.robot.Commands.ZeroHood;
import frc.robot.Commands.ZeroHoodSequence;
import frc.robot.Commands.SetHoodOpenLoop;
import frc.robot.Commands.SetIndexerOpenLoop;
import frc.robot.Commands.SetIndexerVelocity;
import frc.robot.Commands.MoveIntakeToPosition;
import frc.robot.Commands.SetIntakeWheelsOpenLoop;
import frc.robot.Commands.SetIntakeWheelsVelocity;
import frc.robot.Commands.RunTurretOpenLoop;
import frc.robot.Commands.SetChamberOpenLoop;
import frc.robot.Commands.SetChamberVelocity;
import frc.robot.Commands.SetFlywheelVelocity;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Chamber;
import frc.robot.subsystems.ClimberInner;
import frc.robot.subsystems.ClimberOuter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeManifold;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** Maximum translational speed of the robot in meters per second */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    /** Maximum angular rotational rate of the robot in radians per second (3/4 rotation per second) */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /** Field-centric swerve drive request with deadband and open-loop voltage control */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    /** Swerve brake request to lock wheels in place */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    /** Swerve point wheels at request for wheel aiming */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /** Telemetry logger for monitoring robot state */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /** Xbox controller for driver input (port 0) */
    private final CommandXboxController joystick = new CommandXboxController(0);

    /** Swerve drivetrain subsystem */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    /** Inner climber subsystem */
    public final ClimberInner m_climberInner = new ClimberInner();
    /** Outer climber subsystem */
    public final ClimberOuter m_climberOuter = new ClimberOuter();
    /** Flywheel subsystem for shooting mechanisms */
    public final Flywheel m_flywheel = new Flywheel();
    /** Hood subsystem for adjusting shot angle */
    public final Hood m_hood = new Hood();
    /** Intake manifold subsystem for controlling intake position */
    private final IntakeManifold m_intakeManifold = new IntakeManifold();
    /** Intake wheels subsystem for pulling in game pieces */
    private final IntakeWheels m_IntakeWheels = new IntakeWheels();
    /** Indexer subsystem for staging game pieces */
    public final Indexer m_indexer = new Indexer();
    /** Chamber subsystem for pre-shot queuing */
    public final Chamber m_chamber = new Chamber();
    /** Turret subsystem for aiming horizontal rotation */
    public final Turret m_turret = new Turret();

    /**
     * Constructs the RobotContainer and configures all button bindings.
     */
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Configures joystick button bindings and subsystem default commands.
     * 
     * <p>This method sets up:
     * <ul>
     *   <li>Drivetrain default command for field-centric control</li>
     *   <li>Xbox controller button bindings for subsystems</li>
     *   <li>Button triggers for various robot commands</li>
     * </ul>
     * 
     * <p>Button mappings:
     * <ul>
     *   <li>Back button: Intake control</li>
     *   <li>A button: Hub shot sequence</li>
     *   <li>POV Right: Wheel radius calibration</li>
     *   <li>Start button: Reset field-centric heading</li>
     * </ul>
     */
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
        
        // reset the field-centric heading on start press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.back().onTrue(new MoveIntakeToPosition(0, 10, m_intakeManifold).alongWith(new SetIntakeWheelsVelocity(5, 0.1, m_IntakeWheels)));
        // joystick.back().onTrue(new MoveInnerClimberToPosition(0, 0.1, m_climberInner).alongWith(new MoveOuterClimberToPosition(0, 0.1, m_climberOuter)));

        // joystick.x().onTrue(new MoveIntakeToPosition(72, 20, m_intakeManifold).andThen(new SetIntakeWheelsVelocity(30, 80, m_IntakeWheels)))
        //     .onFalse(new SetIntakeWheelsOpenLoop(() -> 0, m_IntakeWheels));

        joystick.a().onTrue(new HubShot(m_flywheel, m_hood, m_turret, m_indexer, m_chamber))
            .onFalse(new RunFlywheelOpenLoop(() -> 0, m_flywheel).alongWith(new SetIndexerVelocity(0, 90, m_indexer).alongWith(new SetChamberVelocity(0, 90, m_chamber))));

        // joystick.povUp().onTrue(new ClimbSequence(m_climberOuter, m_climberInner));

        // joystick.povDown().onTrue(new MoveInnerClimberToPosition(10, 10, m_climberInner).alongWith(new MoveOuterClimberToPosition(10, 10, m_climberOuter)));

        joystick.povRight().onTrue(new WheelRadiusCalibration(drivetrain, drive));
    }

    /**
     * Use this method to pass the autonomous command to the main {@link Robot} class.
     * 
     * <p>Currently returns a placeholder command indicating no autonomous command is configured.
     * Implement autonomous routines here by returning appropriate Command objects.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
