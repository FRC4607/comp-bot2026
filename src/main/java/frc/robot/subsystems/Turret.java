// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Calibrations.TurretCalibrations;
import frc.robot.Constants.TurretConstants;

/** Turret subsystem. */
public class Turret extends SubsystemBase {

    private final TalonFX m_motor;

    private final TalonFXConfiguration m_talonFXConfig;

    private final MotionMagicTorqueCurrentFOC m_request;

    private final CANcoder m_encoder1;
    private final CANcoder m_encoder2;

    private final CANcoderConfiguration m_encoderConfig1;
    private final CANcoderConfiguration m_encoderConfig2;

    private double m_position;

    /** Creates and configures the turret subsystem. */
    public Turret() {

        m_motor = new TalonFX(TurretConstants.kMotorCANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new MotionMagicTorqueCurrentFOC(0);

        m_encoder1 = new CANcoder(TurretConstants.kEncoder1CANID, "kachow");
        m_encoder2 = new CANcoder(TurretConstants.kEncoder2CANID, "kachow");

        m_encoderConfig1 = new CANcoderConfiguration();
        m_encoderConfig2 = new CANcoderConfiguration();

        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = TurretConstants.kRotorToMechanism;

        m_talonFXConfig.ClosedLoopGeneral.ContinuousWrap = false;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_talonFXConfig.Slot0.GravityArmPositionOffset = TurretCalibrations.kGravityOffset;

        m_talonFXConfig.Slot0.kG = TurretCalibrations.kG;
        m_talonFXConfig.Slot0.kS = TurretCalibrations.kS;
        m_talonFXConfig.Slot0.kP = TurretCalibrations.kP;
        m_talonFXConfig.Slot0.kI = TurretCalibrations.kI;
        m_talonFXConfig.Slot0.kD = TurretCalibrations.kD;

        m_talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = TurretCalibrations.kMaxSpeed;
        m_talonFXConfig.MotionMagic.MotionMagicAcceleration = TurretCalibrations.kMaxAcceleration;
        m_talonFXConfig.MotionMagic.MotionMagicJerk = TurretCalibrations.kMaxJerk;

        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretCalibrations.kForwardSoftLimit;

        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretCalibrations.kReverseSoftLimit;

        m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = TurretCalibrations.kMaxAmperage;

        m_motor.getConfigurator().apply(m_talonFXConfig);

        m_encoderConfig1.MagnetSensor.MagnetOffset = TurretCalibrations.kEncoder1Offset;
        m_encoderConfig1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = TurretCalibrations.kEncoder1Discontinuity;
        m_encoderConfig1.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        m_encoder1.getConfigurator().apply(m_encoderConfig1);

        m_encoderConfig2.MagnetSensor.MagnetOffset = TurretCalibrations.kEncoder2Offset;
        m_encoderConfig2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = TurretCalibrations.kEncoder2Discontinuity;
        m_encoderConfig2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder2.getConfigurator().apply(m_encoderConfig2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Turret Encoder Position", getPosition());
        SmartDashboard.putNumber("Turret Motor Position", m_motor.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Encoder 1", m_encoder1.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder 2", m_encoder2.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Updates the position to drive towards.
     *
     * @param newSetpoint The new setpoint, in mechanism rotations.
     */
    public void updateSetpoint(double newSetpoint) {
        m_motor.setControl(m_request.withPosition(((newSetpoint / 360) + 1) % 1));
    }

    /**
     * Runs the turret motor in open loop.
     *
     * @param dutyCycle The power to drive the motor at.
     */
    public void runOpenLoop(double dutyCycle) {
        m_motor.set(dutyCycle);
    }

    /**
     * Gets the current position of the turret in Mechanism rotations.
     * This method uses the Chinese Remainder Theorem to calculate the position of the turret from
     * two encoders with a one-tooth difference in their gears.
     *
     * @return the current position of the turret
     */
    public double getPosition() {
        m_position = (m_encoder2.getAbsolutePosition().getValueAsDouble())
                - (m_encoder1.getAbsolutePosition().getValueAsDouble());

        if (m_position >= 0) {
            return (m_position * 2.35) * 360;
        } else {
            return ((m_position + 1) * 2.35) * 360;
        }
    }

    /**
     * Gets the target position of the mechanism, in degrees.
     *
     * @return The setpoint in degrees.
     */
    public double getSetpoint() {
        return m_motor.getClosedLoopReference().getValueAsDouble() * 360;
    }

    /**
     * Resets the position of the turret to match what the encoders say.
     */
    public void resetsetPosition() {
        if ((getPosition() / 360) > 2) {
            m_motor.setPosition((getPosition() / 360) - 2.35);
        } else {
            m_motor.setPosition(getPosition() / 360);
        }
    }

}
