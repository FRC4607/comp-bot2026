// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ChamberCalibrations;
import frc.robot.Calibrations.IndexerCalibrations;
import frc.robot.Constants.ChamberConstants;
import frc.robot.Constants.IndexerConstants;

/**
 * The Chamber subsystem holds a game piece for a short time before it is shot.
 * It uses a single TalonFX motor with velocity control to manage piece movement.
 */
public class Chamber extends SubsystemBase {

  private final TalonFX m_motor1;
  
  private final TalonFXConfiguration m_talonFXConfig;

  private final VelocityTorqueCurrentFOC m_request;

  /**
   * Creates a new Chamber subsystem and configures the motor.
   */
  public Chamber() {

    m_motor1 = new TalonFX(ChamberConstants.kMotor1CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0).withAcceleration(ChamberCalibrations.kMaxAcceleration);

    m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_talonFXConfig.Slot0.kS = ChamberCalibrations.kS;
    m_talonFXConfig.Slot0.kV = ChamberCalibrations.kV;
    m_talonFXConfig.Slot0.kP = ChamberCalibrations.kP;
    m_talonFXConfig.Slot0.kI = ChamberCalibrations.kI;
    m_talonFXConfig.Slot0.kD = ChamberCalibrations.kD;

    m_motor1.getConfigurator().apply(m_talonFXConfig); 
  }

  /**
   * Updates the chamber velocity setpoint.
   *
   * @param newSetpoint the velocity in rotations per second
   */
  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request.withVelocity(newSetpoint));
  }

  /**
   * Runs the chamber in open-loop mode.
   *
   * @param dutyCycle the duty cycle from -1.0 to 1.0
   */
  public void runOpenLoop(double dutyCycle) {
    m_motor1.set(dutyCycle);
  }

  /**
   * Gets the current velocity of the chamber.
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
