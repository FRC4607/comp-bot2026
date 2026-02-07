// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeWheelCalibrations;
import frc.robot.Constants.IntakeWheelConstants;

public class IntakeWheels extends SubsystemBase {

  private final TalonFX m_motor1;

  private TalonFXConfiguration m_talonFXConfig;

  private VelocityTorqueCurrentFOC m_request;

  private TorqueCurrentFOC m_openLoopRequest;

  /** Creates a new IntakeWheels. */
  public IntakeWheels() {

    m_motor1 = new TalonFX(IntakeWheelConstants.kMotor1CANID, "kachow");

    m_talonFXConfig = new TalonFXConfiguration();

    m_request = new VelocityTorqueCurrentFOC(0)
      .withAcceleration(IntakeWheelCalibrations.kMaxAcceleration);

    m_openLoopRequest = new TorqueCurrentFOC(0);

    m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_talonFXConfig.Slot0.kS = IntakeWheelCalibrations.kS;
    m_talonFXConfig.Slot0.kV = IntakeWheelCalibrations.kV;
    m_talonFXConfig.Slot0.kP = IntakeWheelCalibrations.kP;
    m_talonFXConfig.Slot0.kI = IntakeWheelCalibrations.kI;
    m_talonFXConfig.Slot0.kD = IntakeWheelCalibrations.kD;

    m_talonFXConfig.MotionMagic.MotionMagicAcceleration = IntakeWheelCalibrations.kMaxAcceleration;

    m_talonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = IntakeWheelCalibrations.kMaxAmperage;
    m_talonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = IntakeWheelCalibrations.kMaxAmperage;

    m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_motor1.getConfigurator().apply(m_talonFXConfig);
  }

  public void updateSetpoint(double newSetpoint) {
    m_motor1.setControl(m_request
      .withVelocity(newSetpoint)
    );
  }

  public void setOpenLoop(double amperage) {
    m_motor1.set(amperage);
  }

  public double getVelocity() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
