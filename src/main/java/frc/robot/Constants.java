package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class DrivetrainConstants {
        
    }

    public class TurretConstants {

        /** CAN ID of the motor. */
        public static final int kMotorCANID = 32;

        /** CAN ID of the first encoder. */
        public static final int kEncoder1CANID = 0;

        /** CAN ID of the second encoder. */
        public static final int kEncoder2CANID = 0;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder1ToMechanism = 0;

        /** Gear ratio of # of motor rotor rotations to one sensor rotation. */
        public static final double kEncoder1ToRotor = 0;
        
    }
}
