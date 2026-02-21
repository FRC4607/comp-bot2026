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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeManifoldCalibrations;
import frc.robot.Constants.IntakeManifoldConstants;

public class IntakeManifold extends SubsystemBase {
  /** Creates a new IntakeManifold. */

  private final TalonFX m_motor1;
  private final CANcoder m_encoder;

  private final TalonFXConfiguration m_talonFXConfig;
  private final CANcoderConfiguration m_encoderConfig;

  private final DynamicMotionMagicTorqueCurrentFOC m_request;

  public IntakeManifold() {

    m_motor1 = new TalonFX(IntakeManifoldConstants.kMotor1CANID, "kachow");
    m_encoder = new CANcoder(IntakeManifoldConstants.kEncoderCANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();
    m_encoderConfig = new CANcoderConfiguration();

    m_request = new DynamicMotionMagicTorqueCurrentFOC(
      0.333,
      IntakeManifoldCalibrations.kMaxVelocity,
      IntakeManifoldCalibrations.kMaxAcceleration)
      .withJerk(IntakeManifoldCalibrations.kMaxJerk);

    m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_talonFXConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    m_talonFXConfig.Feedback.SensorToMechanismRatio = IntakeManifoldConstants.kSensorToMechanismRatio;
    m_talonFXConfig.Feedback.RotorToSensorRatio = IntakeManifoldConstants.kRotorToSensorRatio;

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    m_talonFXConfig.Slot0.GravityArmPositionOffset = IntakeManifoldCalibrations.kGravityOffset;
    m_talonFXConfig.Slot0.kG = IntakeManifoldCalibrations.kG;
    m_talonFXConfig.Slot0.kS = IntakeManifoldCalibrations.kS;
    m_talonFXConfig.Slot0.kP = IntakeManifoldCalibrations.kP;
    m_talonFXConfig.Slot0.kI = IntakeManifoldCalibrations.kI;
    m_talonFXConfig.Slot0.kD = IntakeManifoldCalibrations.kD;
    
    m_motor1.getConfigurator().apply(m_talonFXConfig);
    
    m_encoderConfig.MagnetSensor.MagnetOffset = IntakeManifoldCalibrations.kEncoderOffset;
    m_encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = IntakeManifoldCalibrations.kEncoderDiscontinuityPoint;
    m_encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder.getConfigurator().apply(m_encoderConfig);
  }

  /**
   * Sets the setpoint of the mechanism in degrees
   * 
   * @param newSetpoint degrees (0, 360)
   */
  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request.withPosition(newSetpoint / 360));
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
