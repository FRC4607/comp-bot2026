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

    public class HoodCalibrations {

        /** Static Feedforward */
        public static final double kS = 0;

        /** Proportional Gain */
        public static final double kP = 0;

        /** Integral Gain */
        public static final double kI = 0;

        /** Derivative Gain */
        public static final double kD = 0;

        /** Max Stator Current of the mechanism. */
        public static final double kMaxAmperage = 40;
    }
}
