// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Robot Rotation", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Hub State", isHubActive());
    }

    @Override
    public void disabledInit() {
        m_robotContainer.m_turret.resetsetPosition();
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // m_robotContainer.m_intakeArm.updateSetpoint(m_robotContainer.m_intakeArm.getPosition());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isHubActive() {
        if (DriverStation.getAlliance().isEmpty()) {
            return false;
        }
        
        if (isAutonomousEnabled()) {
            return true;
        }

        if (!isTeleopEnabled()) {
            return false;
        }

        double m_matchTime = DriverStation.getMatchTime();
        String m_gameData = DriverStation.getGameSpecificMessage();

        if (m_gameData.isEmpty()) {
            return true;
        }

        boolean m_redActiveFirst;
        if (m_gameData.charAt(0) == 'B') {
            m_redActiveFirst = true;
        } else if (m_gameData.charAt(0) == 'A') {
            m_redActiveFirst = false;
        } else {
            return true;
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (m_matchTime > 130) {
            return true;
        } else if (m_matchTime > 105) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                return true;
            } else {
                return false;
            }
            
        } else if (m_matchTime > 80) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                return false;
            } else {
                return true;
            }

        } else if (m_matchTime > 55) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                return true;
            } else {
                return false;
            }

        } else if (m_matchTime > 30) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                return false;
            } else {
                return true;
            }

        } else {
            return true;
        }
    }
    
}
