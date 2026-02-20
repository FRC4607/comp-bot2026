// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.FlywheelCalibrations;
import frc.robot.Constants.FlywheelConstants;

/**
 * The Flywheel subsystem is the shooting mechanism that launches game pieces at high velocity.
 * It uses two TalonFX motors configured in a leader-follower configuration with velocity control.
 */
public class Flywheel extends SubsystemBase {
  /** Primary motor for the flywheel rotation. */
  private final TalonFX m_motor1;
  /** Secondary motor following the primary motor. */
  private final TalonFX m_motor2;
  
  private TalonFXConfiguration m_talonFXConfig;

  private VelocityTorqueCurrentFOC m_request;

  /**
   * Creates a new Flywheel subsystem and configures the motors.
   */
  public Flywheel() {

    m_motor1 = new TalonFX(FlywheelConstants.kMotor1CANID, "kachow");
    m_motor2 = new TalonFX(FlywheelConstants.kMotor2CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0);

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_talonFXConfig.Slot0.kS = FlywheelCalibrations.kS;
    m_talonFXConfig.Slot0.kV = FlywheelCalibrations.kV;
    m_talonFXConfig.Slot0.kP = FlywheelCalibrations.kP;
    m_talonFXConfig.Slot0.kI = FlywheelCalibrations.kI;
    m_talonFXConfig.Slot0.kD = FlywheelCalibrations.kD;

    m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_motor1.getConfigurator().apply(m_talonFXConfig);
    m_motor2.getConfigurator().apply(m_talonFXConfig);

    m_motor2.setControl(new Follower(FlywheelConstants.kMotor1CANID, MotorAlignmentValue.Opposed));
  }

  /**
   * Updates the flywheel velocity setpoint.
   *
   * @param newSetpoint the velocity in rotations per second
   */
  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withVelocity(newSetpoint)
    );
  }

  /**
   * Runs the flywheel in open-loop mode.
   *
   * @param dutyCycle the duty cycle from -1.0 to 1.0
   */
  public void runOpenLoop(double dutyCycle) {
    m_motor1.set(dutyCycle);
  }

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return the velocity in rotations per second
   */
  public double getVelocity() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
