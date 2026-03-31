package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public class FieldConstants {
        /** Translation of the hub on the blue side. */
        public static final Translation2d kBlueHub = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));

        /** Translation of the hub on the red side. */
        public static final Translation2d kRedHub = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));
    }

    public class DrivetrainConstants {
        
    }

    public class IntakeArmConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 48;

        /** CAN ID of the encoder. */
        public static final int kEncoderCANID = 24;

        /** How many sensor rotations equal one mechanism rotation. */
        public static final double kSensorToMechanismRatio = 2.25;

        /** How many motor rotations equal one sensor rotation. */
        public static final double kRotorToSensorRatio = 23;
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
        public static final int kEncoder1CANID = 50;

        /** CAN ID of the second encoder. */
        public static final int kEncoder2CANID = 51;

        /** Gear ratio of # of motor rotor rotations to one mechanism rotation. */
        public static final double kRotorToMechanism = 10.2;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder1ToMechanism = 6.375;

        /** Gear ratio of # of motor rotor rotations to one sensor rotation. */
        public static final double kEncoder1ToRotor = 1.6;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder2ToMechanism = 6.8;

        /** How far (Degrees) the turret is away from the right plane. */
        public static final double kTurretPositionYaw = 50.29;

        /** How far (Meters) the turret is away from the center of the robot on the XY plane. */
        public static final double kTurretHypotenuse = 0.2744978;
        
    }

    public class HoodConstants {

        /** CAN ID of the motor */
        public static final int kMotorCANID = 52;
    }

    public class FlywheelConstants {

        /** CAN ID of the first motor. */
        // TODO: Conflict with Turret kEncoder2CANID
        public static final int kMotor1CANID = 50;

        /** CAN ID of the second motor. */
        // TODO: Conflict with Turret kEncoder1CANID
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
