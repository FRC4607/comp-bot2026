// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Calibrations.LeftChamberCalibrations;
import frc.robot.Constants.LeftChamberConstants;

/** Chamber subsystem. */
public class LeftChamber extends SubsystemBase {

    private final TalonFX m_motor1;

    private final TalonFXConfiguration m_talonFXConfig;

    private final VelocityTorqueCurrentFOC m_request;

    /** Creates and configures settings for the Chamber. */
    public LeftChamber() {

        // Chamber Motor
        m_motor1 = new TalonFX(LeftChamberConstants.kMotor1CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new VelocityTorqueCurrentFOC(0).withAcceleration(LeftChamberCalibrations.kMaxAcceleration);

        // Feedback settings
        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Gains
        m_talonFXConfig.Slot0.kS = LeftChamberCalibrations.kS;
        m_talonFXConfig.Slot0.kV = LeftChamberCalibrations.kV;
        m_talonFXConfig.Slot0.kP = LeftChamberCalibrations.kP;
        m_talonFXConfig.Slot0.kI = LeftChamberCalibrations.kI;
        m_talonFXConfig.Slot0.kD = LeftChamberCalibrations.kD;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = LeftChamberCalibrations.kMaxAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);
    }

    public void updateSetpoint(double newSetpoint) {
        m_motor1.setControl(m_request.withVelocity(newSetpoint));
    }

    public double getSetpoint() {
        return m_motor1.getClosedLoopReference().getValueAsDouble();
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
