// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeWheelCalibrations;
import frc.robot.Constants.IntakeWheelConstants;

/**
 * The IntakeWheels subsystem controls the spinning wheels that pull game pieces
 * into the robot. It uses a single TalonFX motor with velocity control.
 */
public class IntakeWheels extends SubsystemBase {

  private final TalonFX m_motor1;

  private final TalonFXConfiguration m_talonFXConfig;

  private final VelocityTorqueCurrentFOC m_request;

  /**
   * Creates a new IntakeWheels subsystem and configures the motor.
   */
  public IntakeWheels() {

    m_motor1 = new TalonFX(IntakeWheelConstants.kMotor1CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0).withAcceleration(IntakeWheelCalibrations.kMaxAcceleration);

    m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_talonFXConfig.Slot0.kS = IntakeWheelCalibrations.kS;
    m_talonFXConfig.Slot0.kV = IntakeWheelCalibrations.kV;
    m_talonFXConfig.Slot0.kP = IntakeWheelCalibrations.kP;
    m_talonFXConfig.Slot0.kI = IntakeWheelCalibrations.kI;
    m_talonFXConfig.Slot0.kD = IntakeWheelCalibrations.kD;

    m_motor1.getConfigurator().apply(m_talonFXConfig);
  }

  /**
   * Updates the intake wheel velocity setpoint.
   *
   * @param newSetpoint the velocity in rotations per second
   */
  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request.withVelocity(newSetpoint));
  }

  /**
   * Sets open-loop amperage output for the intake wheels.
   *
   * @param amperage the target amperage
   */
  public void setOpenLoop(double amperage) {
    m_motor1.set(amperage);
  }

  /**
   * Gets the current velocity of the intake wheels.
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
