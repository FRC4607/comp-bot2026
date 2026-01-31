// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turret extends SubsystemBase {

  private final TalonFX m_motor;

  private final TalonFXConfiguration m_talonFXConfig;

  private final MotionMagicTorqueCurrentFOC m_request;

  private final CANcoder m_encoder1;
  private final CANcoder m_encoder2;

  private final CANcoderConfiguration m_encoderConfig1;
  private final CANcoderConfiguration m_encoderConfig2;

  private double m_position;

  /** Creates a new turret. */
  public turret() {

    m_motor = new TalonFX(0, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new MotionMagicTorqueCurrentFOC(0);

    m_encoder1 = new CANcoder(0, "kachow");
    m_encoder2 = new CANcoder(0, "kachow");

    m_encoderConfig1 = new CANcoderConfiguration();
    m_encoderConfig2 = new CANcoderConfiguration();

    m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_talonFXConfig.Feedback.FeedbackRemoteSensorID = 0;
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

    m_motor.getConfigurator().apply(m_talonFXConfig);

    m_encoderConfig1.MagnetSensor.MagnetOffset = 0;
    m_encoderConfig1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    m_encoderConfig1.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder1.getConfigurator().apply(m_encoderConfig1);

    m_encoderConfig2.MagnetSensor.MagnetOffset = 0;
    m_encoderConfig2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    m_encoderConfig2.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder2.getConfigurator().apply(m_encoderConfig2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateSetpoint(double newSetpoint) {

    m_motor.setControl(m_request.withPosition(newSetpoint));

  }

  public double getPosition() {
    m_position = (m_encoder2.getAbsolutePosition().getValueAsDouble()) - (m_encoder1.getAbsolutePosition().getValueAsDouble());

    if (m_position >= 0) {
      return m_position;
    } else {
      return m_position + 1;
    }
  }

  public void resetsetPosition() {
    m_motor.setPosition(getPosition());
  }
}
