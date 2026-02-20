package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical
 * or boolean constants. This should not be used for any other purpose. All string values
 * should be placed in {@link Calibrations} for tuning purposes.
 * 
 * <p>It is advised to statically import this class or one of its inner classes whenever
 * the constants are accessed.
 */
public class Constants {
    /**
     * Drivetrain constants for swerve drive configuration.
     */
    public class DrivetrainConstants {
        
    }

    /**
     * Constants for the intake manifold subsystem (extendable arm).
     */
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

    /**
     * Constants for the intake wheels subsystem.
     */
    public class IntakeWheelConstants {
        /** CAN ID of the intake wheel motor. */
        public static final int kMotor1CANID = 45;
    }

    /**
     * Constants for the indexer subsystem (staging mechanism).
     */
    public class IndexerConstants {

        /** CAN ID of the motor. */
        public static final int kMotor1CANID = 26;
    }

    /**
     * Constants for the chamber subsystem (pre-shot queue).
     */
    public class ChamberConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 55;
    }

    /**
     * Constants for the turret subsystem (horizontal aiming rotation).
     */
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

    /**
     * Constants for the hood subsystem (angle adjustment).
     */
    public class HoodConstants {

        /** CAN ID of the motor */
        public static final int kMotorCANID = 52;
    }

    /**
     * Constants for the flywheel subsystem (shooting mechanism).
     */
    public class FlywheelConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 50;

        /** CAN ID of the second motor. */
        public static final int kMotor2CANID = 51;
    }

    /**
     * Constants for the climber subsystems (inner and outer climb mechanisms).
     */
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
