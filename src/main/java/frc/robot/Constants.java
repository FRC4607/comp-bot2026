package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class DrivetrainConstants {
        
    }

    public class IntakeManifoldConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 0;

        /** CAN ID of the encoder. */
        public static final int kEncoderCANID = 0;

        /** How many sensor rotations equal one mechanism rotation. */
        public static final double kSensorToMechanismRatio = 0;

        /** How many motor rotations equal one sensor rotation. */
        public static final double kRotorToSensorRatio = 0;
    }
}
