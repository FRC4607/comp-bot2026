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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.ClimberCalibrations;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Constants.ClimberConstants;

/** ClimberInner subsystem. */
public class ClimberInner extends SubsystemBase {

    private final TalonFX m_motor1;
    private final TalonFX m_motor2;

    private final TalonFXConfiguration m_talonFXConfig;

    private final MotionMagicTorqueCurrentFOC m_request;
    private final TorqueCurrentFOC m_openLoopRequest;

    /** Creates and configures the Inner Climber. */
    public ClimberInner() {

        m_motor1 = new TalonFX(ClimberConstants.kInnerMotor1CANID, "kachow");
        m_motor2 = new TalonFX(ClimberConstants.kInnerMotor2CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new MotionMagicTorqueCurrentFOC(0);
        m_openLoopRequest = new TorqueCurrentFOC(0);

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        
        // Gains
        m_talonFXConfig.Slot0.kG = ClimberCalibrations.kInnerkG;
        m_talonFXConfig.Slot0.kS = ClimberCalibrations.kInnerkS;
        m_talonFXConfig.Slot0.kP = ClimberCalibrations.kInnerkP;
        m_talonFXConfig.Slot0.kI = ClimberCalibrations.kInnerkI;
        m_talonFXConfig.Slot0.kD = ClimberCalibrations.kInnerkD;

        // Motion Magic settings
        m_talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberCalibrations.kInnerCruiseVelocity;
        m_talonFXConfig.MotionMagic.MotionMagicAcceleration = ClimberCalibrations.kInnerAcceleration;
        m_talonFXConfig.MotionMagic.MotionMagicJerk = ClimberCalibrations.kInnerJerk;

        // Soft limits
        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberCalibrations.kInnerForwardSoftLimit;
        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberCalibrations.kInnerReverseSoftLimit;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = ClimberCalibrations.kMaxAmperage;
        
        m_motor1.getConfigurator().apply(m_talonFXConfig);
        m_motor2.getConfigurator().apply(m_talonFXConfig);

        m_motor2.setControl(new Follower(ClimberConstants.kInnerMotor1CANID, MotorAlignmentValue.Aligned));
    }

    /**
     * Updates the setpoint of the mechanism, in motor rotations.
     *
     * @param newSetpoint The setpoint to drive to in inches (0, ?)
     */
    public void updateSetpoint(double newSetpoint) {
        m_motor1.setControl(m_request
                .withPosition(newSetpoint * (1 / ClimberConstants.kClimbersInchesPerRevolution)));
    }

    /**
     * Runs the motors in open loop control.
     *
     * @param amperage The power to run at, in amps
     */
    public void runOpenLoop(double amperage) {
        m_motor1.setControl(m_openLoopRequest
                .withOutput(amperage));
    }

    /**
     * Gets the position of the mechanism, in inches.
     *
     * @return The current position of the mechanism in inches.
     */
    public double getPosition() {
        return m_motor1.getPosition().getValueAsDouble() / ClimberConstants.kClimbersInchesPerRevolution;
    }

    /**
     * Resets the position of the mechanism to the specified value, in inches.
     *
     * @param position The updated position of the mechanism in inches
     */
    public void setPosition(double position) {
        m_motor1.setPosition(position * (1 / ClimberConstants.kClimbersInchesPerRevolution));
    }

    /**
     * Gets the velocity of the mechanism, in inches per second.
     *
     * @return The current velocity of the mechanism in inches per second.
     */
    public double getVelocity() {
        return m_motor1.getVelocity().getValueAsDouble() * ClimberConstants.kClimbersInchesPerRevolution;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
