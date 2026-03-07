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
        public static final double kMaxAcceleration = 100;

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
        public static final double kMaxAmperage = 40;
        
    }

    /** Calibrations for the chamber. */
    public class ChamberCalibrations {

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
        public static final double kMaxAmperage = 40;
        
    }

    /** Calibrations for the turret. */
    public class TurretCalibrations {

        /** Gravity feedforward. */
        public static final double kG = 0; // 8;

        /** Static feedforward. */
        public static final double kS = 10; // 15;

        /** Proportional gain. */
        public static final double kP = 5500; // 1900;

        /** Integral gain. */
        public static final double kI = 0; // 0;

        /** Derivative gain. */
        public static final double kD = 120; // 75;

        /** Offset of the gravity feedforward. */
        public static final double kGravityOffset = 0; // -0.26;

        /** Max speed of the mechanism. */
        public static final double kMaxSpeed = 10; // 10;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 30; // 40;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Max amperage of the motor. */
        public static final double kMaxAmperage = 40;
        
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

    /** Calibrations for the Hood. */
    public class HoodCalibrations {

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
        public static final double kMaxAmperage = 40;
    }

    /** Calibrations for the flywheel. */
    public class FlywheelCalibrations {

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
        public static final double kMaxAmperage = 40;
    }

    /** Calibrations for the climber. */
    public class ClimberCalibrations {

        /** Gravity feedforward. */
        public static final double kOuterkG = 0;

        /** Static feedforward. */
        public static final double kOuterkS = 1.3;

        /** Proportional Gain.*/
        public static final double kOuterkP = 40;

        /** Integral Gain. */
        public static final double kOuterkI = 0;

        /** Derivative Gain. */
        public static final double kOuterkD = 5;

        /** Max speed. */
        public static final double kOuterCruiseVelocity = 80; // 80;

        /** Max acceleration. */
        public static final double kOuterAcceleration = 400;

        /** Max jerk. */
        public static final double kOuterJerk = 0;

        /** Forward software limit switch - mechanism will not power forwards past this point. */
        public static final double kOuterForwardSoftLimit = 0;

        /** Reverse software limit switch - mechanism will not power backwards past this point. */
        public static final double kOuterReverseSoftLimit = 0;

        /** Gravity feedforward. */
        public static final double kInnerkG = 0;

        /** Static feedforward. */
        public static final double kInnerkS = 1.3;

        /** Proportional Gain.*/
        public static final double kInnerkP = 40;

        /** Integral Gain. */
        public static final double kInnerkI = 0;

        /** Derivative Gain. */
        public static final double kInnerkD = 5;

        /** Max speed. */
        public static final double kInnerCruiseVelocity = 80; // 80;

        /** Max acceleration. */
        public static final double kInnerAcceleration = 400;

        /** Max jerk. */
        public static final double kInnerJerk = 0;

        /** Forward software limit - mechanism will not power forwards past this point. */
        public static final double kInnerForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point. */
        public static final double kInnerReverseSoftLimit = 0;

        /** Current limits for the motors. */
        public static final double kMaxAmperage = 40;
    }

    /** Calibrations for a shot where the robot's left (Opposite Climber) side is against the Hub. */
    public class HubShotCalibrations {

        /** Flywheel Velocity. */
        public static final double kFlywheelVelocity = 45;

        /** Flywheel Velocity Tolerance. */
        public static final double kFlywheelVelocityTolerance = 1;

        /** Hood Angle. */
        public static final double kHoodAngle = 0.65;

        /** Hood Angle Tolerance. */
        public static final double kHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kTurretAngle = 270;

        /** Turret Angle Tolerance. */
        public static final double kTurretAngleTolerance =  1;

        /** Chamber Velocity. */
        public static final double kChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kChamberVelocityTolerance = 90;

        /** Indexer Velocity. */
        public static final double kIndexerVelocity = 30;

        /** Indexer Velocity Tolerance. */
        public static final double kIndexerVelocityTolerance = 90;
    }

    /** 
     * Calibrations for a shot where the the robot is under the trench and the intake
     * is facing the neutral zone and the robot is against the depot-side wall.
     */
    public class DepotTrenchShotCalibrations {

        /** Flywheel Velocity. */
        public static final double kFlywheelVelocity = 56;

        /** Flywheel Velocity Tolerance. */
        public static final double kFlywheelVelocityTolerance = 1;

        /** Hood Angle. */
        public static final double kHoodAngle = 2.2;

        /** Hood Angle Tolerance. */
        public static final double kHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kTurretAngle = 75;

        /** Turret Angle Tolerance. */
        public static final double kTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kChamberVelocityTolerance = 90;

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

        /** Flywheel Velocity. */
        public static final double kFlywheelVelocity = 58;

        /** Flywheel Velocity Tolerance. */
        public static final double kFlywheelVelocityTolerance = 1;

        /** Hood Angle. */
        public static final double kHoodAngle = 2.25;

        /** Hood Angle Tolerance. */
        public static final double kHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kTurretAngle = 277.2;

        /** Turret Angle Tolerance. */
        public static final double kTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kChamberVelocityTolerance = 90;

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

        /** Flywheel Velocity. */
        public static final double kFlywheelVelocity = 73;

        /** Flywheel Velocity Tolerance. */
        public static final double kFlywheelVelocityTolerance = 1;

        /** Hood Angle. */
        public static final double kHoodAngle = 2.25;

        /** Hood Angle Tolerance. */
        public static final double kHoodAngleTolerance = 0.1;

        /** Turret Angle. */
        public static final double kTurretAngle = 48;

        /** Turret Angle Tolerance. */
        public static final double kTurretAngleTolerance = 1;

        /** Chamber Velocity. */
        public static final double kChamberVelocity = 60;

        /** Chamber Velocity Tolerance. */
        public static final double kChamberVelocityTolerance = 90;

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

    /** Calibrations for an L3 Climb. */
    public class ClimbSequenceCalibrations {

        /** Outer climber prep position. */
        public static final double kOuterPrep = 6;

        /** Outer climber prep position tolerance. */
        public static final double kOuterPrepTolerance = 0.25;

        /** Inner climber prep position. */
        public static final double kInnerPrep = 6;

        /** Inner climber prep position tolerance. */
        public static final double kInnerPrepTolerance = 0.25;

        /** Outer climber l1 position. */
        public static final double kOuterPosition = 19;

        /** Outer climber l1 position tolerance. */
        public static final double kOuterPositionTolerance = 0.25;

        /** Inner Climber l1 handoff position. */
        public static final double kInnerHandoffPosition = 20;

        /** Inner Climber l1 handoff tolerance. */
        public static final double kInnerHandoffPositionTolerance = 0.25;

        /** Inner Climber position to allow outer hooks to reach next bar. */
        public static final double kInnerTraversalPosition = 24;

        /** Inner Climber traversal position tolerance. */
        public static final double kInnerTraversalPositionTolerance = 0.25;

        /** Outer Climber position to allow inner hook to reset. */
        public static final double kOuterTraversalPosition = 10;

        /** Outer Climber traversal position tolerance. */
        public static final double kOuterTraversalPositionTolerance = 0.25;
    }

    public class PassWithGyroCalibrations {

        /** Offset of the Turret when tracking. */
        public static final double kTurretAngleOffset = 0;

        /** Angle of the hood to shoot at. */
        public static final double kHoodAngle = 2.25;

        /** Hood angle tolerance. */
        public static final double kHoodTolerance = 0.1;

        /** Number of seconds to continue if hood does not make it to position. */
        public static final double kHoodTimeout = 0.25;

        /** Velocity to run the flywheel at. */
        public static final double kFlywheelVelocity = 75;

        /** Flywheel velocity tolerance. */
        public static final double kFlywheelVelocityTolerance = 1.5;

        /** Number of seconds to continue if flywheel does not make it to velocity. */
        public static final double kFlywheelTimeout = 1;

        /** Velocity to run the chamber at. */
        public static final double kChamberVelocity = 90;

        /** Chamber velocity tolerance. */
        public static final double kChamberVelocityTolerance = 1;

        /** Velocity to run the indexer at. */
        public static final double kIndexerVelocity = 90;

        /** Indexer velocity tolerance. */
        public static final double kIndexerVelocityTolerance = 1;
    }
}
