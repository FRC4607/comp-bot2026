// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ClimberCalibrations;
import frc.robot.Constants.ClimberConstants;

public class ClimberOuter extends SubsystemBase {

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private final TalonFXConfiguration m_talonFXConfig;

  private final MotionMagicTorqueCurrentFOC m_request;
  private final TorqueCurrentFOC m_openLoopRequest;

  /** Creates a new Climber. */
  public ClimberOuter() {

    m_motor1 = new TalonFX(ClimberConstants.kOuterMotor1CANID, "kachow");
    m_motor2 = new TalonFX(ClimberConstants.kOuterMotor2CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new MotionMagicTorqueCurrentFOC(0);
    m_openLoopRequest = new TorqueCurrentFOC(0);

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_talonFXConfig.Slot0.kG = ClimberCalibrations.kOuterkG;
    m_talonFXConfig.Slot0.kS = ClimberCalibrations.kOuterkS;
    m_talonFXConfig.Slot0.kP = ClimberCalibrations.kOuterkP;
    m_talonFXConfig.Slot0.kI = ClimberCalibrations.kOuterkI;
    m_talonFXConfig.Slot0.kD = ClimberCalibrations.kOuterkD;

    m_talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberCalibrations.kOuterCruiseVelocity;
    m_talonFXConfig.MotionMagic.MotionMagicAcceleration = ClimberCalibrations.kOuterAcceleration;
    m_talonFXConfig.MotionMagic.MotionMagicJerk = ClimberCalibrations.kOuterJerk;

    m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberCalibrations.kOuterForwardSoftLimit;

    m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberCalibrations.kOuterReverseSoftLimit;

    m_motor1.getConfigurator().apply(m_talonFXConfig);
    m_motor2.getConfigurator().apply(m_talonFXConfig);

    m_motor2.setControl(new Follower(ClimberConstants.kOuterMotor1CANID, MotorAlignmentValue.Aligned));
  }

  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withPosition(newSetpoint));
  }

  public void runOpenLoop(double amperage) {
    m_motor1.setControl(m_openLoopRequest
      .withOutput(amperage));
  }

  public double getPosition() {
    return m_motor1.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
