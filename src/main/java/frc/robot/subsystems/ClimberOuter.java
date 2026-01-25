// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ClimberCalibrations;
import frc.robot.Constants.ClimberConstants;

public class ClimberOuter extends SubsystemBase {

  private final TalonFX m_outerMotor1;
  private final TalonFX m_outerMotor2;



  private final TalonFXConfiguration m_outerTalonFXConfig;

  private final MotionMagicTorqueCurrentFOC m_request1;

  /** Creates a new Climber. */
  public ClimberOuter() {
    m_outerMotor1 = new TalonFX(ClimberConstants.kOuterMotor1CANID, "kachow");
    m_outerMotor2 = new TalonFX(ClimberConstants.kOuterMotor2CANID, "kachow");

    m_outerTalonFXConfig = new TalonFXConfiguration();

    m_request1 = new MotionMagicTorqueCurrentFOC(0);

    m_outerTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_outerTalonFXConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_outerTalonFXConfig.Slot0.kG = ClimberCalibrations.kOuterkG;
    m_outerTalonFXConfig.Slot0.kS = ClimberCalibrations.kOuterkS;
    m_outerTalonFXConfig.Slot0.kP = ClimberCalibrations.kOuterkP;
    m_outerTalonFXConfig.Slot0.kI = ClimberCalibrations.kOuterkI;
    m_outerTalonFXConfig.Slot0.kD = ClimberCalibrations.kOuterkD;

    m_outerTalonFXConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberCalibrations.kOuterCruiseVelocity;
    m_outerTalonFXConfig.MotionMagic.MotionMagicAcceleration = ClimberCalibrations.kOuterAcceleration;
    m_outerTalonFXConfig.MotionMagic.MotionMagicJerk = ClimberCalibrations.kOuterJerk;

    m_outerTalonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_outerTalonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberCalibrations.kOuterForwardSoftLimit;

    m_outerTalonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_outerTalonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberCalibrations.kOuterReverseSoftLimit;

    m_outerMotor1.getConfigurator().apply(m_outerTalonFXConfig);
    m_outerMotor2.getConfigurator().apply(m_outerTalonFXConfig);

    m_outerMotor2.setControl(new Follower(ClimberConstants.kOuterMotor1CANID, MotorAlignmentValue.Aligned));

    
  }

  public void setOuterClimberSetpoint(double newSetpoint) {
    m_outerMotor1.setControl(m_request1
      .withPosition(newSetpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
