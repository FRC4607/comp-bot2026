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
        public static final double kMaxVelocity = 0;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Gravity feedforward. */
        public static final double kG = 0;

        /** Static feedforward. */
        public static final double kS = 0;

        /** Proportional gain. */
        public static final double kP = 1;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Max amperage of the mechanism. */
        public static final double kMaxAmperage = 40;

        /** Forward soft limit of the mechanism - mechanism will not power forwards past this point */
        public static final double kForwardSoftLimit = 0;

        /** Reverse soft limit of the mechanism - mechanism will not power backwards past this point */
        public static final double kReverseSoftLimit = 0;

        /** Offset of the absolute encoder in rotations. */
        public static final double kEncoderOffset = 0.95;

        /** Wrap-around point of the encoder. */
        public static final double kEncoderDiscontinuityPoint = 0;

    }

    public class IntakeWheelCalibrations {

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Static feedforward. */
        public static final double kS = 0;

        /** Velocity feedforward. */
        public static final double kV = 0.0;

        /** Proportional gain. */
        public static final double kP = 0.0;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Maximum amperage of the motor. */
        public static final double kMaxAmperage = 80;
    }
}
