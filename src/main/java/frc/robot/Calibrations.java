package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class Calibrations {
    public class DrivetrainCalibrations {
         
    }

    /* Calibrations for the extendable part of the intake */
    public class IntakeManifoldCalibrations {

        /** Max Velocity of the mechanism. */
        public static final double kMaxVelocity = 20;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 40;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 9999;

        public static final double kGravityOffset = 0.25;

        /** Gravity feedforward. */
        public static final double kG = 15;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Proportional gain. */
        public static final double kP = 500;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 60;

        /** Max amperage of the mechanism. */
        public static final double kMaxAmperage = 40;

        /** Forward soft limit of the mechanism - mechanism will not power forwards past this point */
        public static final double kForwardSoftLimit = 0.19;

        /** Reverse soft limit of the mechanism - mechanism will not power backwards past this point */
        public static final double kReverseSoftLimit = 0.02;

        /** Offset of the absolute encoder in rotations. */
        public static final double kEncoderOffset = -0.039;

        /** Wrap-around point of the encoder. */
        public static final double kEncoderDiscontinuityPoint = 1;

    }

    public class IntakeWheelCalibrations {

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 50;

        /** Static feedforward. */
        public static final double kS = 2;

        /** Velocity feedforward. */
        public static final double kV = 0.0;

        /** Proportional gain. */
        public static final double kP = 2;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Maximum amperage of the motor. */
        public static final double kMaxAmperage = 80;
    }

    public class TurretCalibrations {

        /** Gravity feedforward. */
        public static final double kG = 0;

        /** Static feedforward. */
        public static final double kS = 0;

        /** Proportional gain. */
        public static final double kP = 0;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Max speed of the mechanism. */
        public static final double kMaxSpeed = 0;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Max amperage of the motor. */
        public static final double kMaxAmperage = 40;
        
        /** Forward software limit - mechanism will not power forwards past this point */
        public static final double kForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point */
        public static final double kReverseSoftLimit = 0;

        /** Offset of the first encoder. */
        public static final double kEncoder1Offset = 0;

        /** Discontinuity point of the first encoder. */
        public static final double kEncoder1Discontinuity = 0;

        /** Offset of the second encoder. */
        public static final double kEncoder2Offset = 0;

        /** Discontinuity point of the second encoder. */
        public static final double kEncoder2Discontinuity = 0;

    }
}
