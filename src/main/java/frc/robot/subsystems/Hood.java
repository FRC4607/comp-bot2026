// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.HoodCalibrations;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  
  private final TalonFXS m_motor;
  
  private final TalonFXSConfiguration m_talonFXSConfig;
  
  private final MotionMagicVoltage m_request;


  /** Creates a new hood. */
  public Hood() {

    m_motor = new TalonFXS(HoodConstants.kMotorCANID, "kachow");

    m_talonFXSConfig = new TalonFXSConfiguration();

    m_request = new MotionMagicVoltage(0);

    m_talonFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    m_talonFXSConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_talonFXSConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    
    m_talonFXSConfig.Slot0.kS = HoodCalibrations.kS;
    m_talonFXSConfig.Slot0.kP = HoodCalibrations.kP;
    m_talonFXSConfig.Slot0.kI = HoodCalibrations.kI;
    m_talonFXSConfig.Slot0.kD = HoodCalibrations.kD;
    
    m_talonFXSConfig.MotionMagic.MotionMagicCruiseVelocity = HoodCalibrations.kMaxSpeed;
    m_talonFXSConfig.MotionMagic.MotionMagicAcceleration = HoodCalibrations.kMaxAcceleration;
    
    m_talonFXSConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_talonFXSConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;

    // m_talonFXSConfig.CurrentLimits.StatorCurrentLimit = HoodCalibrations.kMaxAmperage;

    m_motor.getConfigurator().apply(m_talonFXSConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Voltage", m_motor.getAnalogVoltage().getValueAsDouble());
  }

  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public void updateSetpoint(double newSetpoint) {
    m_motor.setControl(m_request.withPosition(newSetpoint));
    System.out.println("set" + newSetpoint);
  }

  public void runOpenLoop(double dutyCycle) {
    m_motor.set(dutyCycle);
  }

  public void resetPosition(double newPosition) {
    m_motor.setPosition(newPosition);
  }
}
