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
         // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(1.22).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        public static final Distance kWheelRadius = Inches.of(2);

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final Current kSlipCurrent = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        
        public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        public static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(6.21);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        // These are only used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        //Front Left
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.350341796875);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;

        //Front Right
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.385009765625);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;

        //Back Left
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.22412109375);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;

        //Back Right
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.17333984375);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;
    }
}
