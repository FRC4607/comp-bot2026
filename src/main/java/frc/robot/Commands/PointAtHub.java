// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LeftTurretConstants;
import frc.robot.Constants.RightTurretConstants;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LeftFlywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.LeftTurret;
import frc.robot.subsystems.RightFlywheel;
import frc.robot.subsystems.RightHood;
import frc.robot.subsystems.RightTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointAtHub extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private LeftTurret m_leftTurret;
    private LeftHood m_leftHood;
    private LeftFlywheel m_leftFlywheel;
    private RightTurret m_rightTurret;
    private RightHood m_rightHood;
    private RightFlywheel m_rightFlywheel;

    private Translation2d m_targetHubPose;
    private double m_shotOffset;

    private double m_drivetrainAngle;
    private double m_distance;
    private double m_rightDistance;

    private double m_offsetHubX;
    private double m_offsetHubY;

    private ChassisSpeeds m_speeds;

    /** Creates a new PointAtHub. */
    public PointAtHub(CommandSwerveDrivetrain drivetrain, LeftTurret turret, LeftHood leftHood, LeftFlywheel leftFlywheel, RightTurret rightTurret, RightHood rightHood, RightFlywheel rightFlywheel) {
        m_drivetrain = drivetrain;
        m_leftTurret = turret;
        m_leftHood = leftHood;
        m_leftFlywheel = leftFlywheel;
        m_rightTurret = rightTurret;
        m_rightHood = rightHood;
        m_rightFlywheel = rightFlywheel;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftTurret, m_leftHood, m_leftFlywheel, m_rightTurret, m_rightHood, m_rightFlywheel);
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
        m_leftHood.updateSetpoint(2.25);
        m_rightHood.updateSetpoint(2.25);
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var state = m_drivetrain.getState();
        Pose2d pose = state.Pose;
        double poseX = pose.getX();
        double poseY = pose.getY();

        m_drivetrainAngle = pose.getRotation().getDegrees();

        m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, pose.getRotation());

        m_offsetHubX = m_targetHubPose.getX()
            - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult
            * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst));

        m_offsetHubY = m_targetHubPose.getY()
            - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult
            * ((ShootingCalibrations.kVelocityDistanceMult * m_distance) + ShootingCalibrations.kVelocityDistanceConst));

        if (m_offsetHubX > poseX) {
            m_shotOffset = 0;
        } else {
            m_shotOffset = 180;
        }

        double baseAngleRad = Math.toRadians(m_drivetrainAngle + m_shotOffset);
        double baseAngleRad180 = Math.toRadians(m_drivetrainAngle + m_shotOffset + 180);
        double velOffsetX = m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult;
        double velOffsetY = m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult;

        // Left turret aiming
        double leftFlywheelDistMult = SmartDashboard.getNumber(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, ShootingCalibrations.kLeftFlywheelDistanceMult);
        m_distance = computeDistance(poseX, poseY, baseAngleRad, LeftTurretConstants.kLeftTurretPositionYaw, LeftTurretConstants.kLeftTurretHypotenuse);
        m_leftTurret.updateSetpoint(computeTurretSetpoint(poseX, poseY, baseAngleRad180, LeftTurretConstants.kLeftTurretPositionYaw, LeftTurretConstants.kLeftTurretHypotenuse, m_distance, velOffsetX, velOffsetY));
        m_leftFlywheel.updateSetpoint(computeFlywheelSetpoint(poseX, poseY, baseAngleRad, LeftTurretConstants.kLeftTurretPositionYaw, LeftTurretConstants.kLeftTurretHypotenuse, m_distance, velOffsetX, velOffsetY, ShootingCalibrations.kLeftFlywheelConstant, leftFlywheelDistMult));
        SignalLogger.writeDouble("Shooting/FlywheelDistanceMult", leftFlywheelDistMult);

        // Right turret aiming
        double rightFlywheelDistMult = SmartDashboard.getNumber(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey, ShootingCalibrations.kRightFlywheelDistanceMult);
        m_rightDistance = computeDistance(poseX, poseY, baseAngleRad, RightTurretConstants.kRightTurretPositionYaw, RightTurretConstants.kRightTurretHypotenuse);
        m_rightTurret.updateSetpoint(computeTurretSetpoint(poseX, poseY, baseAngleRad180, RightTurretConstants.kRightTurretPositionYaw, RightTurretConstants.kRightTurretHypotenuse, m_rightDistance, velOffsetX, velOffsetY));
        m_rightFlywheel.updateSetpoint(computeFlywheelSetpoint(poseX, poseY, baseAngleRad, RightTurretConstants.kRightTurretPositionYaw, RightTurretConstants.kRightTurretHypotenuse, m_rightDistance, velOffsetX, velOffsetY, ShootingCalibrations.kRightFlywheelConstant, rightFlywheelDistMult));
        SignalLogger.writeDouble("Shooting/RightFlywheelDistanceMult", rightFlywheelDistMult);
    }

    /** Computes the distance from a turret to the hub, without velocity compensation. */
    private double computeDistance(double poseX, double poseY, double baseAngleRad, double turretYaw, double turretHypotenuse) {
        return Math.hypot(
            poseX - (Math.cos(baseAngleRad + turretYaw) * turretHypotenuse) - m_targetHubPose.getX(),
            poseY - (Math.sin(baseAngleRad + turretYaw) * turretHypotenuse) - m_targetHubPose.getY());
    }

    /** Computes the turret setpoint angle in degrees. */
    private double computeTurretSetpoint(double poseX, double poseY, double baseAngleRad180, double turretYaw, double turretHypotenuse, double distance, double velOffsetX, double velOffsetY) {
        double velDistFactor = (ShootingCalibrations.kVelocityDistanceMult * distance) + ShootingCalibrations.kVelocityDistanceConst;
        double numerator = (poseY + (Math.sin(baseAngleRad180 + turretYaw) * turretHypotenuse))
            - (m_targetHubPose.getY() - (velOffsetY * velDistFactor));
        double denominator = (poseX + (Math.cos(baseAngleRad180 + turretYaw) * turretHypotenuse))
            - (m_targetHubPose.getX() - (velOffsetX * velDistFactor));
        return (m_drivetrainAngle + m_shotOffset)
            - ((((Math.atan(numerator / denominator) / Math.PI) * 180)));
    }

    /** Computes the flywheel velocity setpoint based on distance to hub. */
    private double computeFlywheelSetpoint(double poseX, double poseY, double baseAngleRad, double turretYaw, double turretHypotenuse, double distance, double velOffsetX, double velOffsetY, double flywheelConstant, double flywheelDistMult) {
        double velDistFactor = (ShootingCalibrations.kVelocityDistanceMult * distance) + ShootingCalibrations.kVelocityDistanceConst;
        return flywheelConstant + (flywheelDistMult * Math.pow(
            Math.hypot(
                poseX - (Math.cos(baseAngleRad + turretYaw) * turretHypotenuse)
                    - (m_targetHubPose.getX() - (velOffsetX * velDistFactor)),
                poseY - (Math.sin(baseAngleRad + turretYaw) * turretHypotenuse)
                    - (m_targetHubPose.getY() - (velOffsetY * velDistFactor))),
                1));
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
