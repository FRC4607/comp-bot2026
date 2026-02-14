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

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IndexerCalibrations;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private TalonFXConfiguration m_talonFXConfig;

  private VelocityTorqueCurrentFOC m_request;

  /** Creates a new Indexer. */
  public Indexer() {

    m_motor1 = new TalonFX(IndexerConstants.kMotor1CANID, "kachow");
    m_motor2 = new TalonFX(IndexerConstants.kMotor2CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0)
      .withAcceleration(IndexerCalibrations.kMaxAcceleration);

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_talonFXConfig.Slot0.kS = IndexerCalibrations.kS;
    m_talonFXConfig.Slot0.kV = IndexerCalibrations.kV;
    m_talonFXConfig.Slot0.kP = IndexerCalibrations.kP;
    m_talonFXConfig.Slot0.kI = IndexerCalibrations.kI;
    m_talonFXConfig.Slot0.kD = IndexerCalibrations.kD;

    m_talonFXConfig.MotionMagic.MotionMagicAcceleration = IndexerCalibrations.kMaxAcceleration;

    m_talonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = IndexerCalibrations.kMaxAmperage;
    m_talonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = IndexerCalibrations.kMaxAmperage;
    
    m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor1.getConfigurator().apply(m_talonFXConfig); 
    m_motor2.getConfigurator().apply(m_talonFXConfig);
    
    m_motor2.setControl(new Follower(IndexerConstants.kMotor1CANID, MotorAlignmentValue.Aligned));
  }

  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withVelocity(newSetpoint)
    );

  }

  public void runOpenLoop(double dutyCycle) {
    m_motor1.set(dutyCycle);
  }

  public double getVelocity() {
    return m_motor1.getVelocity().getValueAsDouble();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
