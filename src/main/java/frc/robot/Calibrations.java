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

    public class ClimberCalibrations {

        /** Gravity feedforward. */
        public static final double kOuterkG = 0;

        /** Static feedforward. */
        public static final double kOuterkS = 0;

        /** Proportional Gain.*/
        public static final double kOuterkP = 0;

        /** Integral Gain. */
        public static final double kOuterkI = 0;

        /** Derivative Gain. */
        public static final double kOuterkD = 0;

        /** Max speed. */
        public static final double kOuterCruiseVelocity = 0;

        /** Max acceleration. */
        public static final double kOuterAcceleration = 0;

        /** Max jerk. */
        public static final double kOuterJerk = 0;

        /** Forward software limit switch - mechanism will not power forwards past this point. */
        public static final double kOuterForwardSoftLimit = 0;

        /** Reverse software limit switch - mechanism will not power backwards past this point */
        public static final double kOuterReverseSoftLimit = 0;

        /** Gravity feedforward. */
        public static final double kInnerkG = 0;

        /** Static feedforward. */
        public static final double kInnerkS = 0;

        /** Proportional gain. */
        public static final double kInnerkP = 0;

        /** Integral gain. */
        public static final double kInnerkI = 0;

        /** Derivative gain. */
        public static final double kInnerkD = 0;

        /** Max speed. */
        public static final double kInnerCruiseVelocity = 0;

        /** Max acceleration. */
        public static final double kInnerAcceleration = 0;

        /** Max jerk. */
        public static final double kInnerJerk = 0;

        /** Forward software limit - mechanism will not power forwards past this point. */
        public static final double kInnerForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point. */
        public static final double kInnerReverseSoftLimit = 0;
    }
}
