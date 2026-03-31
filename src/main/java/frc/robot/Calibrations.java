package frc.robot;

/** A class where all non-permanent values on the robot are stored. */
public class Calibrations {

    /** Calibrations for the drivetrain. */
    public class DrivetrainCalibrations {
         
    }

    /** Calibrations for the extendable part of the intake. */
    public class IntakeArmCalibrations {

        /** Max Velocity of the mechanism. */
        public static final double kMaxVelocity = 0.5;
        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 3;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 9999;

        public static final double kGravityOffset = 0.25;

        /** Gravity feedforward. */
        public static final double kG = 13;

        /** Static feedforward. */
        public static final double kS = 7;

        /** Proportional gain. */
        public static final double kP = 150;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 90;

        /** Max amperage of the mechanism. */
        public static final double kMaxAmperage = 80;

        /** Forward soft limit of the mechanism - mechanism will not power forwards past this point. */
        public static final double kForwardSoftLimit = 0.19;

        /** Reverse soft limit of the mechanism - mechanism will not power backwards past this point. */
        public static final double kReverseSoftLimit = 0.02;

        /** Offset of the absolute encoder in rotations. */
        public static final double kEncoderOffset = 0.2313;

        /** Wrap-around point of the encoder. */
        public static final double kEncoderDiscontinuityPoint = 0.5;

    }

    /** Calibrations for the intake. */
    public class IntakeWheelCalibrations {

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 90;

        /** Static feedforward. */
        public static final double kS = 24;

        /** Velocity feedforward. */
        public static final double kV = 0.3;

        /** Proportional gain. */
        public static final double kP = 8;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Maximum amperage of the motor. */
        public static final double kMaxAmperage = 80;
    }

    /** Calibrations for the spindexer. */
    public class IndexerCalibrations {

        /** Max acceleration of the Indexer. */
        public static final double kMaxAcceleration = 99999;

        /** Static feedforward. */
        public static final double kS = 1;

        /** Velocity feedforward. */
        public static final double kV = 0;

        /** Proportional gain. */
        public static final double kP = 10;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Amperage limit of the motors. */
        public static final double kMaxAmperage = 80;
        
    }

    /** Calibrations for the leftChamber. */
    public class LeftChamberCalibrations {

        /** Max acceleration of the Indexer. */
        public static final double kMaxAcceleration = 99999;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Velocity feedforward. */
        public static final double kV = 0.2;

        /** Proportional gain. */
        public static final double kP = 10;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Amperage limit of the motors. */
        public static final double kMaxAmperage = 80;
        
    }

    /** Calibrations for the turret. */
    public class LeftTurretCalibrations {

        /** Gravity feedforward. */
        public static final double kG = 0;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Proportional gain. */
        public static final double kP = 10; // 3500

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Offset of the gravity feedforward. */
        public static final double kGravityOffset = 0; // -0.26;

        /** Max speed of the mechanism. */
        public static final double kMaxSpeed = 0.1; // 10;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0.1; // 30;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Max amperage of the motor. */
        public static final double kMaxAmperage = 80;
        
        /** Forward software limit - mechanism will not power forwards past this point. */
        public static final double kForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point. */
        public static final double kReverseSoftLimit = 0;

        /** Offset of the first encoder. */
        public static final double kEncoder1Offset = 0.09;

        /** Discontinuity point of the first encoder. */
        public static final double kEncoder1Discontinuity = 1;

        /** Offset of the second encoder. */
        public static final double kEncoder2Offset = 0.5458;

        /** Discontinuity point of the second encoder. */
        public static final double kEncoder2Discontinuity = 1;

    }

    /** Calibrations for the LeftHood. */
    public class LeftHoodCalibrations {

        /** Static Feedforward. */
        public static final double kS = 0;

        /** Proportional Gain. */
        public static final double kP = 8;

        /** Integral Gain. */
        public static final double kI = 0;

        /** Derivative Gain. */
        public static final double kD = 0;

        /** Maximum velocity of the mechanism. */
        public static final double kMaxSpeed = 20;

        /** Maximum acceleration of the mechanism. */
        public static final double kMaxAcceleration = 9999;

        /** Max Stator Current of the mechanism. */
        public static final double kMaxAmperage = 80;
    }

    /** Calibrations for the leftFlywheel. */
    public class LeftFlywheelCalibrations {

        /** Static Feedforward. */
        public static final double kS = 0.9;

        /** Velocity Feedforward. */
        public static final double kV = 0.05;

        /** Proportional Gain. */
        public static final double kP = 13;

        /** Integral Gain. */
        public static final double kI = 0;

        /** Derivative Gain. */
        public static final double kD = 0;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Current limit of each motor. */
        public static final double kMaxAmperage = 80;
    }

    /** Calibrations for a shot where the robot's left side is against the Hub. */
    public class HubShotCalibrations {

        /** LeftFlywheel Velocity. */
        public static final double kLeftFlywheelVelocity = 45;

        /** LeftFlywheel Velocity Tolerance. */
        public static final double kLeftFlywheelVelocityTolerance = 1;

        /** LeftHood Angle. */
        public static final double kLeftHoodAngle = 0.85;

        /** LeftHood Angle Tolerance. */
        public static final double kLeftHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kLeftTurretAngle = 270;

        /** Turret Angle Tolerance. */
        public static final double kLeftTurretAngleTolerance =  1;

        /** Chamber Velocity. */
        public static final double kLeftChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kLeftChamberVelocityTolerance = 90;

        /** Indexer Velocity. */
        public static final double kIndexerVelocity = 90;

        /** Indexer Velocity Tolerance. */
        public static final double kIndexerVelocityTolerance = 90;
    }

    /** 
     * Calibrations for a shot where the the robot is under the trench and the intake
     * is facing the neutral zone and the robot is against the depot-side wall.
     */
    public class DepotTrenchShotCalibrations {

        /** LeftFlywheel Velocity. */
        public static final double kLeftFlywheelVelocity = 59;

        /** LeftFlywheel Velocity Tolerance. */
        public static final double kLeftFlywheelVelocityTolerance = 1;

        /** LeftHood Angle. */
        public static final double kLeftHoodAngle = 2.2;

        /** LeftHood Angle Tolerance. */
        public static final double kLeftHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kLeftTurretAngle = 80;

        /** Turret Angle Tolerance. */
        public static final double kLeftTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kLeftChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kLeftChamberVelocityTolerance = 90;

        /** Indexer Velocity. */
        public static final double kIndexerVelocity = 90;

        /** Indexer Velocity Tolerance. */
        public static final double kIndexerVelocityTolerance = 90;
    }

    /** 
     * Calibrations for a shot where the the robot is under the trench and the intake
     * is facing the neutral zone and the robot is against the outpost-side wall.
     */
    public class OutpostTrenchShotCalibrations {

        /** LeftFlywheel Velocity. */
        public static final double kLeftFlywheelVelocity = 64;

        /** LeftFlywheel Velocity Tolerance. */
        public static final double kLeftFlywheelVelocityTolerance = 1;

        /** LeftHood Angle. */
        public static final double kLeftHoodAngle = 2.25;

        /** LeftHood Angle Tolerance. */
        public static final double kLeftHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kLeftTurretAngle = 277.2;

        /** Turret Angle Tolerance. */
        public static final double kLeftTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kLeftChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kLeftChamberVelocityTolerance = 90;

        /** Indexer Velocity. */
        public static final double kIndexerVelocity = 90;

        /** Indexer Velocity Tolerance. */
        public static final double kIndexerVelocityTolerance = 90;
    }

    /** 
     * Calibrations for a shot where the the robot is in the 
     * outpost corner and the intake is facing the tower.
     */
    public class OutpostShotCalibrations {

        /** LeftFlywheel Velocity. */
        public static final double kLeftFlywheelVelocity = 81;

        /** LeftFlywheel Velocity Tolerance. */
        public static final double kLeftFlywheelVelocityTolerance = 1;

        /** LeftHood Angle. */
        public static final double kLeftHoodAngle = 2.25;

        /** LeftHood Angle Tolerance. */
        public static final double kLeftHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kLeftTurretAngle = 48;

        /** Turret Angle Tolerance. */
        public static final double kLeftTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kLeftChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kLeftChamberVelocityTolerance = 90;

        /** Indexer Velocity. */
        public static final double kIndexerVelocity = 90;

        /** Indexer Velocity Tolerance. */
        public static final double kIndexerVelocityTolerance = 90;
    }

    /** Calibrations for an L1 Climb. */
    public class ChinUpCalibrations {

        /** Position to go to to get off the ground. */
        public static final double kOuterChinUpPosition = 12;

        /** Tolerance for error in the chinup position. */
        public static final double kOuterChinUpTolerance = 0.25;
    }

    /** Calibrations for a shot which uses the gyro to pass fuel towards the driver station. */
    public class PassWithGyroCalibrations {

        /** Offset of the Turret when tracking. */
        public static final double kLeftTurretAngleOffset = 0;

        /** Angle of the leftHood to shoot at. */
        public static final double kLeftHoodAngle = 2.25;

        /** LeftHood angle tolerance. */
        public static final double kLeftHoodTolerance = 0.1;

        /** Number of seconds to continue if leftHood does not make it to position. */
        public static final double kLeftHoodTimeout = 0.25;

        /** Velocity to run the leftFlywheel at. */
        public static final double kLeftFlywheelVelocity = 75;

        /** LeftFlywheel velocity tolerance. */
        public static final double kLeftFlywheelVelocityTolerance = 1.5;

        /** Number of seconds to continue if leftFlywheel does not make it to velocity. */
        public static final double kLeftFlywheelTimeout = 1;

        /** Velocity to run the left chamber at. */
        public static final double kLeftChamberVelocity = 90;

        /** Chamber velocity tolerance. */
        public static final double kLeftChamberVelocityTolerance = 1;

        /** Velocity to run the indexer at. */
        public static final double kIndexerVelocity = 90;

        /** Indexer velocity tolerance. */
        public static final double kIndexerVelocityTolerance = 1;
    }

    /** Calibrations for the right chamber. */
    public class RightChamberCalibrations {

        /** Max acceleration of the Indexer. */
        public static final double kMaxAcceleration = 99999;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Velocity feedforward. */
        public static final double kV = 0.2;

        /** Proportional gain. */
        public static final double kP = 10;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Amperage limit of the motors. */
        public static final double kMaxAmperage = 80;

    }

    /** Calibrations for the right turret. */
    public class RightTurretCalibrations {

        /** Gravity feedforward. */
        public static final double kG = 0;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Proportional gain. */
        public static final double kP = 10;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Offset of the gravity feedforward. */
        public static final double kGravityOffset = 0;

        /** Max speed of the mechanism. */
        public static final double kMaxSpeed = 0.1;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0.1;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Max amperage of the motor. */
        public static final double kMaxAmperage = 80;

        /** Forward software limit - mechanism will not power forwards past this point. */
        public static final double kForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point. */
        public static final double kReverseSoftLimit = 0;

        /** Offset of the first encoder. */
        // TODO: Calibrate encoder offset
        public static final double kEncoder1Offset = 0;

        /** Discontinuity point of the first encoder. */
        public static final double kEncoder1Discontinuity = 1;

        /** Offset of the second encoder. */
        // TODO: Calibrate encoder offset
        public static final double kEncoder2Offset = 0;

        /** Discontinuity point of the second encoder. */
        public static final double kEncoder2Discontinuity = 1;

    }

    /** Calibrations for the right hood. */
    public class RightHoodCalibrations {

        /** Static Feedforward. */
        public static final double kS = 0;

        /** Proportional Gain. */
        public static final double kP = 8;

        /** Integral Gain. */
        public static final double kI = 0;

        /** Derivative Gain. */
        public static final double kD = 0;

        /** Maximum velocity of the mechanism. */
        public static final double kMaxSpeed = 20;

        /** Maximum acceleration of the mechanism. */
        public static final double kMaxAcceleration = 9999;

        /** Max Stator Current of the mechanism. */
        public static final double kMaxAmperage = 80;
    }

    /** Calibrations for the right flywheel. */
    public class RightFlywheelCalibrations {

        /** Static Feedforward. */
        public static final double kS = 0.9;

        /** Velocity Feedforward. */
        public static final double kV = 0.05;

        /** Proportional Gain. */
        public static final double kP = 13;

        /** Integral Gain. */
        public static final double kI = 0;

        /** Derivative Gain. */
        public static final double kD = 0;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Current limit of each motor. */
        public static final double kMaxAmperage = 80;
    }

    /** Calibrations for a shoot from anywhere command. */
    public class ShootingCalibrations {
        
        /** 
         * Add this much to the predicted leftFlywheel velocity. 
         * Increasing this increases shot distance at all ranges.
         */
        public static final double kLeftFlywheelConstant = 22.5;

        
        /** 
         * Add this much per meter of distance from the Hub, after velocity offset.
         * Increasing this increases shot distance at long range.
         */
        public static final double kLeftFlywheelDistanceMult = 13; // 11
        public static final String kLeftFlywheelDistanceMultPrefKey = "Left Flywheel Distance Multiplier";

        /**
         * Add this much to the predicted right flywheel velocity.
         * Increasing this increases shot distance at all ranges.
         */
        public static final double kRightFlywheelConstant = 22.5;

        /**
         * Add this much per meter of distance from the Hub, after velocity offset.
         * Increasing this increases shot distance at long range.
         */
        public static final double kRightFlywheelDistanceMult = 13;
        public static final String kRightFlywheelDistanceMultPrefKey = "Right Flywheel Distance Multiplier";

        /**
         * Correction amount for velocity offset. 
         * Increasing will make it counter velocity more overall.
         */
        public static final double kVelocityOffsetMult = 0.2;

        /** 
         * Multiplier for distance, to use in velocity offset. Increasing this value will make it
         * counter velocity harder at long range.
         */
        public static final double kVelocityDistanceMult = 1.5; // 1.1
        
        /** 
         * Constant to add to the distance, after multiplier, to use in the velocity offset.
         * Increasing this value will make it counter velocity more at close range.
         */
        public static final double kVelocityDistanceConst = 1.5;
    }
}
