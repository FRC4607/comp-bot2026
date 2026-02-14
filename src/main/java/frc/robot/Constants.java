package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class DrivetrainConstants {
        
    }

    public class IntakeManifoldConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 48;

        /** CAN ID of the encoder. */
        public static final int kEncoderCANID = 24;

        /** How many sensor rotations equal one mechanism rotation. */
        public static final double kSensorToMechanismRatio = 2;

        /** How many motor rotations equal one sensor rotation. */
        public static final double kRotorToSensorRatio = 5;
    }

    public class IntakeWheelConstants {
        public static final int kMotor1CANID = 45;
    }

    public class TurretConstants {

        /** CAN ID of the motor. */
        public static final int kMotorCANID = 0;

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
