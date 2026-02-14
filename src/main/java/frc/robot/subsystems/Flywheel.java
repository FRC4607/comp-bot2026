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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.FlywheelCalibrations;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private final TalonFX m_motor1;
  private final TalonFX m_motor2;
  
  private TalonFXConfiguration m_talonFXConfig;

  private VelocityTorqueCurrentFOC m_request;

  /** Creates a new IntakeWheels. */
  public Flywheel() {

    m_motor1 = new TalonFX(FlywheelConstants.kMotor1CANID, "kachow");
    m_motor2 = new TalonFX(FlywheelConstants.kMotor2CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0);

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_talonFXConfig.Slot0.kS = FlywheelCalibrations.kS;
    m_talonFXConfig.Slot0.kV = FlywheelCalibrations.kV;
    m_talonFXConfig.Slot0.kP = FlywheelCalibrations.kP;
    m_talonFXConfig.Slot0.kI = FlywheelCalibrations.kI;
    m_talonFXConfig.Slot0.kD = FlywheelCalibrations.kD;

    m_talonFXConfig.MotionMagic.MotionMagicAcceleration = FlywheelCalibrations.kMaxAcceleration;

    m_talonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = FlywheelCalibrations.kMaxAmperage;
    m_talonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = FlywheelCalibrations.kMaxAmperage;

    m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor1.getConfigurator().apply(m_talonFXConfig);
    m_motor2.getConfigurator().apply(m_talonFXConfig);

    m_motor2.setControl(new Follower(FlywheelConstants.kMotor1CANID, MotorAlignmentValue.Opposed));
  }

  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withVelocity(newSetpoint)
    );
  }

  public void runOpenLoop(double dutyCycle) {
    m_motor1.set(dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
