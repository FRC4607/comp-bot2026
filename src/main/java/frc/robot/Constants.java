package frc.robot;

public class Constants {
    public class DrivetrainConstants {
        
    }

    public class IntakeManifoldConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 48;

        /** CAN ID of the encoder. */
        public static final int kEncoderCANID = 24;

        /** How many sensor rotations equal one mechanism rotation. */
        public static final double kSensorToMechanismRatio = 3;

        /** How many motor rotations equal one sensor rotation. */
        public static final double kRotorToSensorRatio = 5;
    }

    public class IntakeWheelConstants {
        public static final int kMotor1CANID = 45;
    }

    public class IndexerConstants {

        /** CAN ID of the motor. */
        public static final int kMotor1CANID = 26;
    }

    public class ChamberConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 55;
    }

    public class TurretConstants {

        /** CAN ID of the motor. */
        public static final int kMotorCANID = 32;

        /** CAN ID of the first encoder. */
        public static final int kEncoder1CANID = 51;

        /** CAN ID of the second encoder. */
        public static final int kEncoder2CANID = 50;

        /** Gear ratio of # of motor rotor rotations to one mechanism rotation. */
        public static final double kRotorToMechanism = 10.2;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder1ToMechanism = 6.375;

        /** Gear ratio of # of motor rotor rotations to one sensor rotation. */
        public static final double kEncoder1ToRotor = 1.6;

        /** Offset for the first encoder. */
        public static final double kEncoder1Offset = 0.521973;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder2ToMechanism = 6.8;

        /** Offset for the second encoder. */
        public static final double kEncoder2Offset = 0.450928;
        
    }

    public class HoodConstants {

        /** CAN ID of the motor */
        public static final int kMotorCANID = 52;
    }

    public class FlywheelConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 50;

        /** CAN ID of the second motor. */
        public static final int kMotor2CANID = 51;
    }

    public class ClimberConstants {

        /** CAN ID of the first outer chain motor. */
        public static final int kOuterMotor1CANID = 23;

        /** Can ID of the second outer chain motor. */
        public static final int kOuterMotor2CANID = 6;

        /** CAN ID of the first inner chain motor. */
        public static final int kInnerMotor1CANID = 15;

        /** CAN ID of the second inner chain motor. */
        public static final int kInnerMotor2CANID = 56;

        /** Inches of chain travel per motor revolution. */
        public static final double kClimbersInchesPerRevolution = 0.19864;

        // 5.625 inches per revolution of outer sprocket
    }
}
