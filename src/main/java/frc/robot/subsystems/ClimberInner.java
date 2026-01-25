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

public class ClimberInner extends SubsystemBase {
  
  private final TalonFX m_innerMotor1;
  private final TalonFX m_innerMotor2;

  private final TalonFXConfiguration m_innerTalonFXConfig;

  private final MotionMagicTorqueCurrentFOC m_request2;
  /** Creates a new InnerClimber. */
  public ClimberInner() {

    m_innerMotor1 = new TalonFX(ClimberConstants.kInnerMotor1CANID, "kachow");
    m_innerMotor2 = new TalonFX(ClimberConstants.kInnerMotor2CANID, "kachow");

    m_innerTalonFXConfig = new TalonFXConfiguration();

    m_request2 = new MotionMagicTorqueCurrentFOC(0);

    m_innerTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_innerTalonFXConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_innerTalonFXConfig.Slot0.kG = ClimberCalibrations.kInnerkG;
    m_innerTalonFXConfig.Slot0.kS = ClimberCalibrations.kInnerkS;
    m_innerTalonFXConfig.Slot0.kP = ClimberCalibrations.kInnerkP;
    m_innerTalonFXConfig.Slot0.kI = ClimberCalibrations.kInnerkI;
    m_innerTalonFXConfig.Slot0.kD = ClimberCalibrations.kInnerkD;

    m_innerTalonFXConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberCalibrations.kInnerCruiseVelocity;
    m_innerTalonFXConfig.MotionMagic.MotionMagicAcceleration = ClimberCalibrations.kInnerAcceleration;
    m_innerTalonFXConfig.MotionMagic.MotionMagicJerk = ClimberCalibrations.kInnerJerk;

    m_innerTalonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_innerTalonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberCalibrations.kInnerForwardSoftLimit;

    m_innerTalonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_innerTalonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberCalibrations.kInnerReverseSoftLimit;

    m_innerMotor1.getConfigurator().apply(m_innerTalonFXConfig);
    m_innerMotor2.getConfigurator().apply(m_innerTalonFXConfig);

    m_innerMotor2.setControl(new Follower(ClimberConstants.kInnerMotor1CANID, MotorAlignmentValue.Aligned));
  }

  public void setInnerClimberSetpoint(double newSetpoint) {
    m_innerMotor1.setControl(m_request2
      .withPosition(newSetpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
