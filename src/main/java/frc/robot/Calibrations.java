package frc.robot;

public class Calibrations {
    public class DrivetrainCalibrations {

        
         
    }

    /* Calibrations for the extendable part of the intake */
    public class IntakeManifoldCalibrations {

        /** Max Velocity of the mechanism. */
        public static final double kMaxVelocity = 20;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 40;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 9999;

        public static final double kGravityOffset = 0.25;

        /** Gravity feedforward. */
        public static final double kG = 15;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Proportional gain. */
        public static final double kP = 700;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 60;

        /** Max amperage of the mechanism. */
        public static final double kMaxAmperage = 40;

        /** Forward soft limit of the mechanism - mechanism will not power forwards past this point */
        public static final double kForwardSoftLimit = 0.19;

        /** Reverse soft limit of the mechanism - mechanism will not power backwards past this point */
        public static final double kReverseSoftLimit = 0.02;

        /** Offset of the absolute encoder in rotations. */
        public static final double kEncoderOffset = -0.318604;

        /** Wrap-around point of the encoder. */
        public static final double kEncoderDiscontinuityPoint = 1;

    }

    public class IntakeWheelCalibrations {

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 99999;

        /** Static feedforward. */
        public static final double kS = 10;

        /** Velocity feedforward. */
        public static final double kV = 0.2;

        /** Proportional gain. */
        public static final double kP = 20;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 0;

        /** Maximum amperage of the motor. */
        public static final double kMaxAmperage = 80;
    }

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

    public class TurretCalibrations {

        /** Gravity feedforward. */
        public static final double kG = 8;

        /** Static feedforward. */
        public static final double kS = 15;

        /** Proportional gain. */
        public static final double kP = 2200;

        /** Integral gain. */
        public static final double kI = 0;

        /** Derivative gain. */
        public static final double kD = 100;

        /** Offset of the gravity feedforward. */
        public static final double kGravityOffset = -0.26;

        /** Max speed of the mechanism. */
        public static final double kMaxSpeed = 10;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 40;

        /** Max jerk of the mechanism. */
        public static final double kMaxJerk = 0;

        /** Max amperage of the motor. */
        public static final double kMaxAmperage = 40;
        
        /** Forward software limit - mechanism will not power forwards past this point */
        public static final double kForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point */
        public static final double kReverseSoftLimit = 0;

        /** Offset of the first encoder. */
        public static final double kEncoder1Offset = -0.425;

        /** Discontinuity point of the first encoder. */
        public static final double kEncoder1Discontinuity = 1;

        /** Offset of the second encoder. */
        public static final double kEncoder2Offset = -0.303;

        /** Discontinuity point of the second encoder. */
        public static final double kEncoder2Discontinuity = 1;

    }

    public class HoodCalibrations {

        /** Static Feedforward */
        public static final double kS = 0;

        /** Proportional Gain */
        public static final double kP = 8;

        /** Integral Gain */
        public static final double kI = 0;

        /** Derivative Gain */
        public static final double kD = 0;

        /** Maximum velocity of the mechanism. */
        public static final double kMaxSpeed = 20;

        /** Maximum acceleration of the mechanism. */
        public static final double kMaxAcceleration = 9999;

        /** Max Stator Current of the mechanism. */
        public static final double kMaxAmperage = 40;
    }

    public class FlywheelCalibrations {

        /** Static Feedforward. */
        public static final double kS = 0;

        /** Velocity Feedforward. */
        public static final double kV = 0;

        /** Proportional Gain. */
        public static final double kP = 12.5;

        /** Integral Gain. */
        public static final double kI = 0;

        /** Derivative Gain. */
        public static final double kD = 0;

        /** Max acceleration of the mechanism. */
        public static final double kMaxAcceleration = 0;

        /** Current limit of each motor. */
        public static final double kMaxAmperage = 40;
    }

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
        public static final double kOuterCruiseVelocity = 80;

        /** Max acceleration. */
        public static final double kOuterAcceleration = 400;

        /** Max jerk. */
        public static final double kOuterJerk = 0;

        /** Forward software limit switch - mechanism will not power forwards past this point. */
        public static final double kOuterForwardSoftLimit = 0;

        /** Reverse software limit switch - mechanism will not power backwards past this point */
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
        public static final double kInnerCruiseVelocity = 80;

        /** Max acceleration. */
        public static final double kInnerAcceleration = 400;

        /** Max jerk. */
        public static final double kInnerJerk = 0;

        /** Forward software limit - mechanism will not power forwards past this point. */
        public static final double kInnerForwardSoftLimit = 0;

        /** Reverse software limit - mechanism will not power backwards past this point. */
        public static final double kInnerReverseSoftLimit = 0;
    }

    /** Calibrations for a shot where the robot's left (Opposite Climber) side is against the Hub. */
    public class HubShotCalibrations {

        /** Flywheel Velocity */
        public static final double kFlywheelVelocity = 45;

        /** Flywheel Velocity Tolerance */
        public static final double kFlywheelVelocityTolerance = 1;

        /** Hood Angle */
        public static final double kHoodAngle = 0.65;

        /** Hood Angle Tolerance */
        public static final double kHoodAngleTolerance = 0.1;

        /** Turret Angle */
        public static final double kTurretAngle = 0.25;

        /** Turret Angle Tolerance */
        public static final double kTurretAngleTolerance = 0.05;

        /** Chamber Velocity */
        public static final double kChamberVelocity = 60;

        /** Chamber Velocity Tolerance */
        public static final double kChamberVelocityTolerance = 90;

        /** Indexer Velocity */
        public static final double kIndexerVelocity = 90;

        /** Indexer Velocity Tolerance */
        public static final double kIndexerVelocityTolerance = 90;
    }
}
