// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  private final TalonFXConfiguration m_talonFXConfig1;
  private final TalonFXConfiguration m_talonFXConfig2;

  private final MotionMagicTorqueCurrentFOC m_request1;
  private final MotionMagicTorqueCurrentFOC m_request2;
  /** Creates a new Climber. */
  public Climber() {
    m_motor1 = new TalonFX(0, "kachow");
    m_motor2 = new TalonFX(0, "kachow");

    m_talonFXConfig1 = new TalonFXConfiguration();
    m_talonFXConfig2 = new TalonFXConfiguration();

    m_request1 = new MotionMagicTorqueCurrentFOC(0);
    m_request2 = new MotionMagicTorqueCurrentFOC(0);

    m_talonFXConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_talonFXConfig1.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_talonFXConfig1.Slot0.kG = 0;
    m_talonFXConfig1.Slot0.kS = 0;
    m_talonFXConfig1.Slot0.kP = 0;
    m_talonFXConfig1.Slot0.kI = 0;
    m_talonFXConfig1.Slot0.kD = 0;

    m_talonFXConfig1.MotionMagic.MotionMagicCruiseVelocity = 0;
    m_talonFXConfig1.MotionMagic.MotionMagicAcceleration = 0;
    m_talonFXConfig1.MotionMagic.MotionMagicJerk = 0;

    m_talonFXConfig1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_talonFXConfig1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

    m_talonFXConfig1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_talonFXConfig1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_motor1.getConfigurator().apply(m_talonFXConfig1);

    m_talonFXConfig2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_talonFXConfig2.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_talonFXConfig2.Slot0.kG = 0;
    m_talonFXConfig2.Slot0.kS = 0;
    m_talonFXConfig2.Slot0.kP = 0;
    m_talonFXConfig2.Slot0.kI = 0;
    m_talonFXConfig2.Slot0.kD = 0;

    m_talonFXConfig2.MotionMagic.MotionMagicCruiseVelocity = 0;
    m_talonFXConfig2.MotionMagic.MotionMagicAcceleration = 0;
    m_talonFXConfig2.MotionMagic.MotionMagicJerk = 0;

    m_talonFXConfig2.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_talonFXConfig2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;

    m_talonFXConfig2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_talonFXConfig2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_motor2.getConfigurator().apply(m_talonFXConfig2);
  }

  public void setOuterClimberSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request1
      .withPosition(newSetpoint));
  }

  public void setInnerClimberSetpoint(double newSetpoint) {
    m_motor2.setControl(m_request2
      .withPosition(newSetpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
