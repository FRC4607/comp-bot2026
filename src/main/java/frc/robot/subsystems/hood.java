// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class hood extends SubsystemBase {
  
  private TalonFXS m_motor;
  
  private TalonFXSConfiguration m_talonFXSConfig;
  
  private TorqueCurrentFOC m_request;
  private PIDController m_pid;
  private double m_setpoint = 0;

  /** Creates a new hood. */
  public hood() {

    m_motor = new TalonFXS(0);

    m_talonFXSConfig = new TalonFXSConfiguration();

    m_request = new TorqueCurrentFOC(0);

    m_pid = new PIDController(0, 0, 0);

    m_talonFXSConfig.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;

    m_talonFXSConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_talonFXSConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_talonFXSConfig.CurrentLimits.StatorCurrentLimit = 40;

    m_motor.getConfigurator().apply(m_talonFXSConfig);

  }

  public void updateSetpoint(double newSetpoint) {
    m_setpoint = newSetpoint;
  }

  @Override
  public void periodic() {
    
    m_motor.setControl(
      m_request.withOutput(
        m_pid.calculate(m_motor.getAnalogVoltage().getValueAsDouble(), m_setpoint)));

    // This method will be called once per scheduler run
  }
}
