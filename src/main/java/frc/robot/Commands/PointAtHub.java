// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointAtHub extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private Turret m_turret;
    private Hood m_hood;
    private Flywheel m_flywheel;

    private Translation2d m_targetHubPose;
    private double m_shotOffset;

    private double m_drivetrainAngle;
    private double m_distance;

    private double m_offsetHubX;
    private double m_offsetHubY;

    private ChassisSpeeds m_speeds;

    /** Creates a new PointAtHub. */
    public PointAtHub(CommandSwerveDrivetrain drivetrain, Turret turret, Hood hood, Flywheel flywheel) {
        m_drivetrain = drivetrain;
        m_turret = turret;
        m_hood = hood;
        m_flywheel = flywheel;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_turret, m_hood, m_flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Red) {
            m_targetHubPose = FieldConstants.kRedHub;
            // m_shotOffset = 180;
            
            System.out.println("Aiming at Red Hub");
        } else {
            m_targetHubPose = FieldConstants.kBlueHub;
            // m_shotOffset = 0;
            System.out.println("Aiming at Blue Hub");
        }
        
        m_drivetrainAngle = m_drivetrain.getState().Pose.getRotation().getDegrees();
        m_hood.updateSetpoint(2.25);
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrainAngle = m_drivetrain.getState().Pose.getRotation().getDegrees();

        m_speeds = m_drivetrain.getState().Speeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, m_drivetrain.getState().Pose.getRotation());

        m_offsetHubX = m_targetHubPose.getX() 
            - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst));

        m_offsetHubY = m_targetHubPose.getY() 
            - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst));

        if (m_offsetHubX > m_drivetrain.getState().Pose.getX()) {
            m_shotOffset = 0;
        } else {
            m_shotOffset = 180;
        }

        m_distance = Math.hypot(
                m_drivetrain.getState().Pose.getX() 
                - (Math.cos((((m_drivetrainAngle + m_shotOffset) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse) 
                - m_targetHubPose.getX(), 
                m_drivetrain.getState().Pose.getY() 
                - (Math.sin((((m_drivetrainAngle + m_shotOffset) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse)
                - m_targetHubPose.getY());

        m_turret.updateSetpoint(
            (m_drivetrainAngle + m_shotOffset)

            /* ArcTangent to find field relative turret angle */
            - ((((Math.atan(((m_drivetrain.getState().Pose.getY() 
                + (Math.sin((((m_drivetrainAngle + m_shotOffset + 180) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse)) 
                - (m_targetHubPose.getY() - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst)))) 
            / (m_drivetrain.getState().Pose.getX() 
                + (Math.cos((((m_drivetrainAngle + m_shotOffset + 180) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse) 
                - (m_targetHubPose.getX() - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst))))) 
            / 3.14159) * 180))));

        m_flywheel.updateSetpoint(ShootingCalibrations.kFlywheelConstant + (SmartDashboard.getNumber(ShootingCalibrations.kFlywheelDistanceMultPrefKey, ShootingCalibrations.kFlywheelDistanceMult) * Math.pow(
            Math.hypot(
                m_drivetrain.getState().Pose.getX() 
                - (Math.cos((((m_drivetrainAngle + m_shotOffset) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse) 
                - (m_targetHubPose.getX() - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst))), 
                m_drivetrain.getState().Pose.getY() 
                - (Math.sin((((m_drivetrainAngle + m_shotOffset) / 180) * 3.14159) + TurretConstants.kTurretPositionYaw) * TurretConstants.kTurretHypotenuse)
                - (m_targetHubPose.getY() - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst)))),
                1)));


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
