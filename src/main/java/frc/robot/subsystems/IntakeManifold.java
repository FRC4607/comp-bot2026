// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeManifold extends SubsystemBase {
  /** Creates a new IntakeManifold. */

  private final TalonFX m_motor1;
  private final CANcoder m_encoder;

  private TalonFXConfiguration m_talonFXConfig;
  private CANcoderConfiguration m_encoderConfig;

  private final DynamicMotionMagicTorqueCurrentFOC m_request;

  public IntakeManifold() {

    m_motor1 = new TalonFX(0, "kachow");
    m_encoder = new CANcoder(0, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();
    m_encoderConfig = new CANcoderConfiguration();

    m_request = new DynamicMotionMagicTorqueCurrentFOC(
      0,
      0,
      0)
      .withJerk(0);

    m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_talonFXConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_talonFXConfig.Feedback.SensorToMechanismRatio = 0;
    m_talonFXConfig.Feedback.RotorToSensorRatio = 0;

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    m_talonFXConfig.Slot0.kG = 0;
    m_talonFXConfig.Slot0.kS = 0;
    m_talonFXConfig.Slot0.kP = 0;
    m_talonFXConfig.Slot0.kI = 0;
    m_talonFXConfig.Slot0.kD = 0;

    m_talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    m_talonFXConfig.MotionMagic.MotionMagicAcceleration = 0;
    m_talonFXConfig.MotionMagic.MotionMagicJerk = 0;

    m_talonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 0;
    m_talonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;

    m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor1.getConfigurator().apply(m_talonFXConfig);

    m_encoderConfig.MagnetSensor.MagnetOffset = 0;
    m_encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    m_encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder.getConfigurator().apply(m_encoderConfig);
  }

  /**
   * Sets the setpoint of the mechanism in degrees
   * 
   * @param newSetpoint degrees (0, 360)
   */
  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withPosition(newSetpoint));
  }

  /**
   * @return the position of the mechanism in degrees
   */
  public double getPosition() {
    return m_encoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
